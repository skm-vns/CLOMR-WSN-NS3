/*
 * CLOMR: Cross-Layer Optimization of MAC Scheduling and Energy-Aware Routing
 * for Mobile Data Collection in Large-Scale Sparse WSNs.
 * Place this file in your ns-3 "scratch/" directory and run with:
 * ./waf --run smart-wsn
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lr-wpan-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/log.h"
#include <vector>
#include <cmath>
#include <fstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("ClomrSimulation");

// ==========================================
// Global Variables & CSV Streams
// ==========================================
std::ofstream g_mdcMetrics;
std::ofstream g_energyMetrics;
bool g_mdcInRange = false; // Cross-layer global flag

// ==========================================
// Cross-Layer Information Base (CLIB) Structure
// ==========================================
struct ClibData {
    double sojournDistance;
    double linkQuality;
    uint32_t queueOccupancy;     
    double macThroughput;        
    bool urgencyFlag;            
};

// ==========================================
// CLOMR Custom Application
// ==========================================
class ClomrApplication : public Application 
{
public:
    enum Role { SENSOR, CLUSTER_HEAD, MDC };

    ClomrApplication() : m_role(SENSOR), m_queue(0), m_macThroughput(50.0), m_totalDataCollected(0.0), m_residualEnergy(10000.0) {}
    virtual ~ClomrApplication() {}

    void SetRole(Role role) { m_role = role; }
    void SetClusterHeadId(uint32_t chId) { m_chId = chId; }
    void AddClusterMember(Ptr<Node> member) { m_clusterMembers.push_back(member); }
    ClibData GetClib() { return m_clib; }

protected:
    virtual void StartApplication(void) {
        if (m_role == SENSOR) {
            Simulator::Schedule(Seconds(1.0), &ClomrApplication::GenerateData, this);
            Simulator::Schedule(Seconds(1.0), &ClomrApplication::UpdateEnergy, this);
        } else if (m_role == CLUSTER_HEAD) {
            Simulator::Schedule(Seconds(1.0), &ClomrApplication::UpdateClib, this);
            Simulator::Schedule(Seconds(1.0), &ClomrApplication::UpdateEnergy, this);
        } else if (m_role == MDC) {
            Simulator::Schedule(Seconds(0.5), &ClomrApplication::MdcControlLoop, this);
        }
    }

    virtual void StopApplication(void) {}

private:
    Role m_role;
    uint32_t m_chId;
    std::vector<Ptr<Node>> m_clusterMembers;
    ClibData m_clib;

    uint32_t m_queue; 
    double m_macThroughput;
    double m_totalDataCollected;
    double m_residualEnergy; // Mathematical energy tracking

    // Parameters from the paper
    const double v_min = 0.5;
    const double v_max = 30.0;
    const double beta = 0.5;        
    const double r_detect = 150.0;  
    const uint32_t N_req = 1000;    
    const double alpha_mac = 0.8;   

    // Sensor Logic
    void GenerateData() {
        m_queue += 1; 
        m_residualEnergy -= 0.05; // Transmission energy cost
        Simulator::Schedule(Seconds(5.0), &ClomrApplication::GenerateData, this);
    }

    // Mathematical Energy Modeling (Replaces ns-3 basic energy model)
    void UpdateEnergy() {
        if (m_role == MDC) return; // MDC doesn't track battery here

        double energyDrain = 0.0;

        if (m_role == SENSOR) {
            if (g_mdcInRange) {
                // CLOMR Paper Claim: TDMA sleep slots during urgency phase save energy
                energyDrain = 0.002; 
            } else {
                // Normal CSMA-CA idle listening
                energyDrain = 0.015; 
            }
        } else if (m_role == CLUSTER_HEAD) {
            if (g_mdcInRange) {
                // High throughput TDMA uploading to MDC consumes more CH energy
                energyDrain = 0.080; 
            } else {
                // Normal cluster aggregation
                energyDrain = 0.025;
            }
        }

        m_residualEnergy -= energyDrain;

        // Log Energy to CSV every 5 seconds to keep file size clean
        if (g_energyMetrics.is_open() && (int(Simulator::Now().GetSeconds()) % 5 == 0)) {
            std::string roleStr = (m_role == SENSOR) ? "SENSOR" : "CH";
            g_energyMetrics << Simulator::Now().GetSeconds() << "," 
                            << GetNode()->GetId() << "," 
                            << roleStr << ","
                            << m_residualEnergy << "\n";
        }

        Simulator::Schedule(Seconds(1.0), &ClomrApplication::UpdateEnergy, this);
    }

    // Cluster Head Logic
    void UpdateClib() {
        m_clib.queueOccupancy = m_queue;
        m_clib.macThroughput = m_macThroughput;

        if (g_mdcInRange) {
            // Apply Eq 6: TDMA Urgency Schedule Adaptation
            m_clib.macThroughput = alpha_mac * 100.0; 
            NS_LOG_INFO("Time: " << Simulator::Now().GetSeconds() << "s | CH Node " << GetNode()->GetId() << " entered URGENCY MAC mode. Throughput boosted.");
        } else {
            m_clib.macThroughput = 50.0; 
        }

        Simulator::Schedule(Seconds(1.0), &ClomrApplication::UpdateClib, this);
    }

    // MDC Logic: The CLOMR-Adapt Algorithm
    void MdcControlLoop() {
        Ptr<MobilityModel> mdcMob = GetNode()->GetObject<MobilityModel>();
        Vector mdcPos = mdcMob->GetPosition();

        Vector chPos(500.0, 500.0, 0.0); 
        double d_k = CalculateDistance(mdcPos, chPos);

        Vector vel = mdcMob->GetVelocity();
        double currentSpeed = sqrt(vel.x * vel.x + vel.y * vel.y);
        if (currentSpeed == 0) currentSpeed = 5.0; 

        // Pre-Contact Phase
        if (d_k < r_detect) {
            g_mdcInRange = true; // Trigger network-wide urgency cross-layer adaptation
            
            NS_LOG_INFO("Time: " << Simulator::Now().GetSeconds() << "s | MDC in Pre-Contact Phase. Distance to CH: " << d_k << "m");

            double lq = std::min(1.0, 100.0 / d_k); 
            double R_MAC_k = 80.0; 
            double R_CH_MDC = 250.0 * lq; 
            
            double R_eff = std::min(R_CH_MDC, R_MAC_k);

            // Accumulate data collected (runs every 0.5s)
            m_totalDataCollected += (R_eff * 0.5);

            // Compute v_opt_k -> Equation 5
            double v_opt = (lq * d_k * (R_eff + beta * currentSpeed)) / (N_req + beta * d_k);
            
            if (v_opt > v_max) v_opt = v_max;
            if (v_opt < v_min) v_opt = v_min;

            currentSpeed = v_opt;

            NS_LOG_INFO("CLOMR Algorithm Output: Optimizing MDC Speed to " << v_opt << " m/s");

            Ptr<ConstantVelocityMobilityModel> cvMob = mdcMob->GetObject<ConstantVelocityMobilityModel>();
            if (cvMob) cvMob->SetVelocity(Vector(currentSpeed, 0.0, 0.0));
        } else {
            g_mdcInRange = false;
        }

        // Log Metrics to CSV
        if (g_mdcMetrics.is_open()) {
            g_mdcMetrics << Simulator::Now().GetSeconds() << ","
                         << d_k << ","
                         << currentSpeed << ","
                         << m_totalDataCollected << "\n";
        }

        Simulator::Schedule(Seconds(0.5), &ClomrApplication::MdcControlLoop, this);
    }
};

// ==========================================
// Simulation Main Runner
// ==========================================
int main(int argc, char *argv[])
{
    LogComponentEnable("ClomrSimulation", LOG_LEVEL_INFO);

    // Initialize Global CSV Output Files
    g_mdcMetrics.open("clomr_mdc_metrics.csv", std::ios::out);
    if (g_mdcMetrics.is_open()) {
        g_mdcMetrics << "Time(s),Distance_to_CH(m),MDC_Speed(m/s),Total_Data_Collected(packets)\n";
    }

    g_energyMetrics.open("clomr_energy_metrics.csv", std::ios::out);
    if (g_energyMetrics.is_open()) {
        g_energyMetrics << "Time(s),NodeID,Role,RemainingEnergy(J)\n";
    }

    uint32_t numSensors = 100;
    uint32_t numChs = 10;
    
    NodeContainer sensorNodes;
    sensorNodes.Create(numSensors);
    
    NodeContainer chNodes;
    chNodes.Create(numChs);
    
    NodeContainer mdcNode;
    mdcNode.Create(1);

    MobilityHelper mobility;
    
    mobility.SetPositionAllocator("ns3::RandomRectanglePositionAllocator",
                                  "X", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=1200.0]"),
                                  "Y", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=1200.0]"));
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(sensorNodes);
    mobility.Install(chNodes);

    // Deploy MDC closer to CH so data collection starts quicker (at 300 instead of 0)
    Ptr<ListPositionAllocator> mdcAlloc = CreateObject<ListPositionAllocator>();
    mdcAlloc->Add(Vector(300.0, 500.0, 0.0));
    mobility.SetPositionAllocator(mdcAlloc);
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(mdcNode);
    mdcNode.Get(0)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(5.0, 0.0, 0.0));

    LrWpanHelper lrWpanHelper;
    NetDeviceContainer sensorDevices = lrWpanHelper.Install(sensorNodes);
    NetDeviceContainer chDevices = lrWpanHelper.Install(chNodes);
    NetDeviceContainer mdcDevice = lrWpanHelper.Install(mdcNode);

    lrWpanHelper.EnablePcapAll("clomr-trace");

    // Install Apps on Sensors
    for (uint32_t i = 0; i < sensorNodes.GetN(); ++i) {
        Ptr<ClomrApplication> app = CreateObject<ClomrApplication>();
        app->SetRole(ClomrApplication::SENSOR);
        sensorNodes.Get(i)->AddApplication(app);
        app->SetStartTime(Seconds(1.0));
        app->SetStopTime(Seconds(86400.0)); 
    }

    // Install Apps on CHs
    for (uint32_t i = 0; i < chNodes.GetN(); ++i) {
        Ptr<ClomrApplication> app = CreateObject<ClomrApplication>();
        app->SetRole(ClomrApplication::CLUSTER_HEAD);
        app->SetClusterHeadId(i);
        chNodes.Get(i)->AddApplication(app);
        app->SetStartTime(Seconds(1.0));
        app->SetStopTime(Seconds(86400.0));
    }

    // Install App on MDC
    Ptr<ClomrApplication> mdcApp = CreateObject<ClomrApplication>();
    mdcApp->SetRole(ClomrApplication::MDC);
    mdcNode.Get(0)->AddApplication(mdcApp);
    mdcApp->SetStartTime(Seconds(1.0));
    mdcApp->SetStopTime(Seconds(86400.0));

    NS_LOG_INFO("Starting Simulation...");
    Simulator::Stop(Seconds(100.0)); 
    Simulator::Run();
    Simulator::Destroy();

    if (g_mdcMetrics.is_open()) g_mdcMetrics.close();
    if (g_energyMetrics.is_open()) g_energyMetrics.close();

    NS_LOG_INFO("Simulation Complete.");
    return 0;
}

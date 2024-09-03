#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/wifi-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/yans-wifi-phy.h"
#include "ns3/ssid.h"


using namespace ns3;

/*
 * Source: NS3 Simulation Projects Team.
 * https://ns3simulation.com/how-to-implement-5g-network-slicing-in-ns3/
 */

int main(int argc, char *argv[])
{
    // 3. Create network topology:
    NodeContainer coreNodes;
    coreNodes.Create(1); // Create core network node
    NodeContainer gnbNodes;
    gnbNodes.Create(2); // Create two gNBs
    NodeContainer ueNodes;
    ueNodes.Create(4); // Create four UEs

    // Set up mobility model:
    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                  "MinX", DoubleValue(0.0),
                                  "MinY", DoubleValue(0.0),
                                  "DeltaX", DoubleValue(100.0),
                                  "DeltaY", DoubleValue(100.0),
                                  "GridWidth", UintegerValue(2),
                                  "LayoutType", StringValue("RowFirst"));

    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(coreNodes);
    mobility.Install(gnbNodes);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(ueNodes);

    // 5. Set up point-to-point links for gNB to CN communication
    PointToPointHelper pointToPoint;
    pointToPoint.SetDeviceAttribute("DataRate", StringValue("10Gbps"));
    pointToPoint.SetChannelAttribute("Delay", StringValue("2ms"));
    NetDeviceContainer coreGnbDevices;
    for (uint32_t i = 0; i < gnbNodes.GetN(); ++i) {
        NetDeviceContainer link = pointToPoint.Install(NodeContainer(coreNodes.Get(0), gnbNodes.Get(i)));
        coreGnbDevices.Add(link);
    }

    // 6. Install Internet stack to CN:
    InternetStackHelper internet;
    internet.Install(coreNodes);
    internet.Install(gnbNodes);
    internet.Install(ueNodes);

    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer coreGnbInterfaces = address.Assign(coreGnbDevices);

    // 7. Set up WiFi for UE to gNB communication
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper wifiPhy;//= YansWifiPhyHelper::Default();
    wifiPhy.SetChannel(wifiChannel.Create());
    WifiHelper wifi;
    wifi.SetStandard(WifiStandard::WIFI_STANDARD_80211n);//"WIFI_PHY_STANDARD_80211n" does not exist!
    WifiMacHelper wifiMac;

    Ssid ssid1 = Ssid("ns-3-ssid-1");
    Ssid ssid2 = Ssid("ns-3-ssid-2");
    wifiMac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid1));
    
    NetDeviceContainer ueDevices1 = wifi.Install(wifiPhy, wifiMac, ueNodes.Get(0));
    ueDevices1.Add(wifi.Install(wifiPhy, wifiMac, ueNodes.Get(1)));
    wifiMac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid2));

    NetDeviceContainer ueDevices2 = wifi.Install(wifiPhy, wifiMac, ueNodes.Get(2));
    ueDevices2.Add(wifi.Install(wifiPhy, wifiMac, ueNodes.Get(3)));
    wifiMac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid1));

    NetDeviceContainer gnbDevices1 = wifi.Install(wifiPhy, wifiMac, gnbNodes.Get(0));
    wifiMac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid2));

    NetDeviceContainer gnbDevices2 = wifi.Install(wifiPhy, wifiMac, gnbNodes.Get(1));

    // 8. Assign IP addresses to the UE devices:
    Ipv4InterfaceContainer ueInterfaces1 = address.Assign(ueDevices1);
    Ipv4InterfaceContainer ueInterfaces2 = address.Assign(ueDevices2);

    // 9. Create network slices:
    uint16_t port1 = 8000;
    uint16_t port2 = 8001;

    OnOffHelper onoff1("ns3::UdpSocketFactory", Address(InetSocketAddress(ueInterfaces1.GetAddress(1), port1)));
    onoff1.SetConstantRate(DataRate("5Mbps"));
    ApplicationContainer apps1 = onoff1.Install(ueNodes.Get(0));
    apps1.Start(Seconds(1.0));
    apps1.Stop(Seconds(10.0));

    PacketSinkHelper sink1("ns3::UdpSocketFactory", Address(InetSocketAddress(Ipv4Address::GetAny(), port1)));
    apps1 = sink1.Install(ueNodes.Get(1));
    apps1.Start(Seconds(0.0));
    apps1.Stop(Seconds(10.0));

    OnOffHelper onoff2("ns3::UdpSocketFactory", Address(InetSocketAddress(ueInterfaces2.GetAddress(1), port2)));
    onoff2.SetConstantRate(DataRate("10Mbps"));
    ApplicationContainer apps2 = onoff2.Install(ueNodes.Get(2));
    apps2.Start(Seconds(1.0));
    apps2.Stop(Seconds(10.0));

    PacketSinkHelper sink2("ns3::UdpSocketFactory", Address(InetSocketAddress(Ipv4Address::GetAny(), port2)));
    apps2 = sink2.Install(ueNodes.Get(3));
    apps2.Start(Seconds(0.0));
    apps2.Stop(Seconds(10.0));

    // 10. Run the simulation:
    Simulator::Stop(Seconds(10.0));
    Simulator::Run();
    Simulator::Destroy();
    
    return 0;
}
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/stats-module.h"
#include "ns3/wifi-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/yans-wifi-phy.h"
#include "ns3/ssid.h"

#include <fstream>

/**
 * REF:
 * 1. https://www.nsnam.org/docs/models/html/flow-monitor.html
 * 2. examples/tutorial/seventh.cc
 */

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("LogTestScript");

class RandomApp : public Application
{
  public:
    RandomApp();
    ~RandomApp() override;

    /**
     * Register this type.
     * \return The TypeId.
     */
    static TypeId GetTypeId();

    /**
     * Setup the socket.
     * \param socket The socket.
     * \param address The destination address.
     * \param packetSize The packet size to transmit.
     * \param nPackets The number of packets to transmit.
     * \param dataRate the data rate to use.
     */
    void Setup(Ptr<Socket> socket,
               Address address,
               uint32_t packetSize,
               uint32_t nPackets,
               DataRate dataRate);

  private:
    void StartApplication() override;
    void StopApplication() override;

    /// Schedule a new transmission.
    void ScheduleTx();
    /// Send a packet.
    void SendPacket();

    Ptr<Socket> m_socket;   //!< The transmission socket.
    Address m_peer;         //!< The destination address.
    uint32_t m_packetSize;  //!< The packet size.
    uint32_t m_nPackets;    //!< The number of packets to send.
    DataRate m_dataRate;    //!< The data rate to use.
    EventId m_sendEvent;    //!< Send event.
    bool m_running;         //!< True if the application is running.
    uint32_t m_packetsSent; //!< The number of packets sent.
};

RandomApp::RandomApp()
    : m_socket(nullptr),
      m_peer(),
      m_packetSize(0),
      m_nPackets(0),
      m_dataRate(0),
      m_sendEvent(),
      m_running(false),
      m_packetsSent(0)
{
}

RandomApp::~RandomApp()
{
    m_socket = nullptr;
}

/* static */
TypeId
RandomApp::GetTypeId()
{
    static TypeId tid = TypeId("RandomApp")
        .SetParent<Application>()
        .SetGroupName("Tutorial")
        .AddConstructor<RandomApp>();
    return tid;
}

void
RandomApp::Setup(Ptr<Socket> socket,
                 Address address,
                 uint32_t packetSize,
                 uint32_t nPackets,
                 DataRate dataRate)
{
    m_socket = socket;
    m_peer = address;
    m_packetSize = packetSize;
    m_nPackets = nPackets;
    m_dataRate = dataRate;
}

void
RandomApp::StartApplication()
{
    m_running = true;
    m_packetsSent = 0;
    m_socket->Bind();
    m_socket->Connect(m_peer);
    SendPacket();
}

void
RandomApp::StopApplication()
{
    m_running = false;

    if (m_sendEvent.IsPending())
    {
        Simulator::Cancel(m_sendEvent);
    }

    if (m_socket)
    {
        m_socket->Close();
    }
}

void
RandomApp::SendPacket()
{
    Ptr<Packet> packet = Create<Packet>(m_packetSize);
    m_socket->Send(packet);

    if (++m_packetsSent < m_nPackets)
    {
        ScheduleTx();
    }
}

void
RandomApp::ScheduleTx()
{
    if (m_running)
    {
        Time tNext(Seconds(m_packetSize * 8 / static_cast<double>(m_dataRate.GetBitRate())));
        m_sendEvent = Simulator::Schedule(tNext, &RandomApp::SendPacket, this);
    }
}

/**
 * Rx drop callback
 *
 * \param file The output PCAP file.
 * \param p The dropped packet.
 */
static void
RxDrop(Ptr<PcapFileWrapper> file, Ptr<const Packet> p)
{
    NS_LOG_UNCOND("RxDrop at " << Simulator::Now().GetSeconds());
    file->Write(Simulator::Now(), p);
}

/**
 * TODO: Make associate helper class
 */

int main(int argc, char *argv[])
{
    int n_slice = 2;
    int n_ue_slice = 2;
    double stop_time = 10.0;
    double cleanup_time = 0.0;

    CommandLine cmd(__FILE__);
//    cmd.AddValue("n_slice", "The Number of Slices", n_slice);
//    cmd.AddValue("n_ue_slice", "The Number of UEs in Each Slice", n_ue_slice);
    cmd.AddValue("stop_time", "Application Runtime", stop_time);
    cmd.AddValue("cleanup_time", "Cleanup Time After Application Stops", stop_time);
    cmd.Parse(argc, argv);

    // Create network topology: 
    NodeContainer coreNodes;
    coreNodes.Create(1);
    NodeContainer gnbNodes;
    gnbNodes.Create(n_slice);
    NodeContainer ueNodes;
    ueNodes.Create(n_ue_slice + 2);

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

    // Set up point-to-point links for gNB to CN communication
    PointToPointHelper pointToPoint;
    pointToPoint.SetDeviceAttribute("DataRate", StringValue("10Gbps"));
    pointToPoint.SetChannelAttribute("Delay", StringValue("2ms"));
    NetDeviceContainer coreGnbDevices;
    for (uint32_t i = 0; i < gnbNodes.GetN(); ++i) {
        NetDeviceContainer link = pointToPoint.Install(NodeContainer(coreNodes.Get(0), gnbNodes.Get(i)));
        coreGnbDevices.Add(link);
    }

    // Install Internet stack to CN:
    InternetStackHelper internet;
    internet.Install(coreNodes);
    internet.Install(gnbNodes);
    internet.Install(ueNodes);

    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer coreGnbInterfaces = address.Assign(coreGnbDevices);

    // Set up WiFi for UE to gNB communication
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

    // Assign IP addresses to the UE devices:
    Ipv4InterfaceContainer ueInterfaces1 = address.Assign(ueDevices1);
    Ipv4InterfaceContainer ueInterfaces2 = address.Assign(ueDevices2);

    // Flow monitor:
    Ptr<FlowMonitor> flowMonitor;
    FlowMonitorHelper flowHelper;
    flowMonitor = flowHelper.InstallAll();

    // Create network slices:
    uint16_t port1 = 8000;
    uint16_t port2 = 8001;
    uint16_t sinkPort = 8080;

    for (int i = 0; i < n_ue_slice; ++i) {
        // Install sink app on each UE 
        Address sinkAddr(InetSocketAddress(ueInterfaces1.GetAddress(i), sinkPort));
        PacketSinkHelper packetSinkHelper("ns3::TcpSocketFactory", sinkAddr);
        ApplicationContainer apprx = packetSinkHelper.Install(ueNodes.Get(i));
        apprx.Start(Seconds(0.0));
        apprx.Stop(Seconds(stop_time));

        // Create TCP socket
        Ptr<Socket> ns3TcpSocket = Socket::CreateSocket(coreNodes.Get(0), TcpSocketFactory::GetTypeId());

        // Create application
        Ptr<RandomApp> apptx = CreateObject<RandomApp>();
        apptx->Setup(ns3TcpSocket, sinkAddr, 1040, 1000, DataRate("1Mbps"));
        coreNodes.Get(0)->AddApplication(apptx);
        apptx->SetStartTime(Seconds(0.));
        apptx->SetStopTime(Seconds(stop_time));
/*
        RandomAppHelper packetGenHelper = RandomAppHelper(
            "ns3::TcpSocketFactory",
            InetSocketAddress(coreGnbInterfaces.GetAddress(i), port1));
        packetGenHelper.SetAttribute("Delay", StringValue ("Constant:2.5"));
        packetGenHelper.SetAttribute("Size", StringValue ("Constant:2100"));
        ApplicationContainer apptx = packetGenHelper.Install(coreNodes.Get(0));
        apptx.Start(Seconds(0.0));
        apptx.Stop(Seconds(stop_time));
*/
    }

    OnOffHelper onoff2("ns3::UdpSocketFactory", Address(InetSocketAddress(ueInterfaces2.GetAddress(1), port2)));
    onoff2.SetConstantRate(DataRate("10Mbps"));
    ApplicationContainer apps2 = onoff2.Install(ueNodes.Get(2));
    apps2.Start(Seconds(0.0));
    apps2.Stop(Seconds(stop_time));

    PacketSinkHelper sink2("ns3::UdpSocketFactory", Address(InetSocketAddress(Ipv4Address::GetAny(), sinkPort)));
    apps2 = sink2.Install(ueNodes.Get(3));
    apps2.Start(Seconds(0.0));
    apps2.Stop(Seconds(stop_time));

    // Logger on
    PcapHelper pcapHelper;
    Ptr<PcapFileWrapper> file =
        pcapHelper.CreateFile("rxdrop.pcap", std::ios::out, PcapHelper::DLT_PPP);
    coreGnbDevices.Get(1)->TraceConnectWithoutContext("[gNB]PhyTxDrop", MakeBoundCallback(&RxDrop, file));
    ueDevices1.Get(0)->TraceConnectWithoutContext("[UE]PhyRxDrop", MakeBoundCallback(&RxDrop, file));

    Simulator::Stop(Seconds(stop_time+cleanup_time));
    Simulator::Run();

    flowMonitor->SerializeToXmlFile("NameOfFile.xml", true, true);

    Simulator::Destroy();
    
    return 0;
}
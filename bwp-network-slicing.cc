#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/config-store-module.h"
#include "ns3/config-store.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/ideal-beamforming-algorithm.h"
#include "ns3/internet-apps-module.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/log.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-mac-scheduler-tdma-rr.h"
#include "ns3/nr-module.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/point-to-point-module.h"

#include <iostream>
#include <chrono>
#include <ctime>    

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("PreliminaryNetworkSlicingSimulation");

int
main(int argc, char* argv[])
{
    const uint8_t numCcs = 3;

    uint16_t ueNumPergNb [] = {2, 3, 4};
    uint16_t numFlowsUe = 1;

    double centralFrequencyBand = 28e9;
    double bandwidthBand = 3e9;

    // general(non-contiguous) cc setting
    double centralFrequencyCc [] = {27e9, 28e9, 29e9};
    double bandwidthCc [] = {400e6, 300e6, 300e6};
    uint16_t numerologyCc [] = {3, 4, 3};


    std::string pattern =
        "F|F|F|F|F|F|F|F|F|F|"; // Pattern can be e.g. "DL|S|UL|UL|DL|DL|S|UL|UL|DL|"
    double totalTxPower = 8;
    bool cellScan = false;
    double beamSearchAngleStep = 10.0;

    bool udpFullBuffer = false;
    uint32_t udpPacketSize [] = {1252, 1252, 1252};   // packet size in bytes
    uint32_t lambda [] = {1000, 1000, 1000};
    /*
    uint32_t udpPacketSize [] = {(uint32_t)(45000000./8./60.),
                                 (uint32_t)(30000000./8./60.),
                                 (uint32_t)(10000000./8./30.)};   // packet size in bytes
    uint32_t lambda [] = {60, 60, 30};
    */
    bool logging = true;

    // commencing...
    auto start = std::chrono::system_clock::now();
    std::time_t start_time = std::chrono::system_clock::to_time_t(start);
    std::string simTag = std::ctime(&start_time);
    std::string outputDir = "./";

    // unit: seconds
    double simTime = 1;          
    double udpAppStartTime = 0.1;

    CommandLine cmd(__FILE__);

    cmd.AddValue("simTime", "Simulation time", simTime);
    cmd.AddValue("ueNumPergNb0", "The number of UE per gNb of type 1 in multiple-ue topology", ueNumPergNb[0]);
    cmd.AddValue("ueNumPergNb1", "The number of UE per gNb of type 2 in multiple-ue topology", ueNumPergNb[1]);
    cmd.AddValue("ueNumPergNb2", "The number of UE per gNb of type 3 in multiple-ue topology", ueNumPergNb[2]);
    cmd.AddValue("centralFrequencyBand",
                 "The system frequency to be used in band 1",
                 centralFrequencyBand);
    cmd.AddValue("bandwidthBand", "The system bandwidth to be used in band 1", bandwidthBand);
    cmd.AddValue("centralFrequencyCc0",
                 "The system frequency to be used in CC 0",
                 centralFrequencyCc[0]);
    cmd.AddValue("bandwidthCc0", "The system bandwidth to be used in CC 0", bandwidthCc[0]);
    cmd.AddValue("centralFrequencyCc1",
                 "The system frequency to be used in CC 1",
                 centralFrequencyCc[1]);
    cmd.AddValue("bandwidthCc1", "The system bandwidth to be used in CC 1", bandwidthCc[1]);
    cmd.AddValue("centralFrequencyCc2",
                 "The system frequency to be used in CC 1",
                 centralFrequencyCc[2]);
    cmd.AddValue("bandwidthCc2", "The system bandwidth to be used in CC 2", bandwidthCc[2]);
    cmd.AddValue("numerologyCc0", "Numerlogy to be used in CC 0, BWP 0", numerologyCc[0]);
    cmd.AddValue("numerologyCc1", "Numerlogy to be used in CC 0, BWP 1", numerologyCc[1]);
    cmd.AddValue("numerologyCc2", "Numerlogy to be used in CC 1, BWP 0", numerologyCc[2]);
    cmd.AddValue("tddPattern",
                 "LTE TDD pattern to use (e.g. --tddPattern=DL|S|UL|UL|UL|DL|S|UL|UL|UL|)",
                 pattern);
    cmd.AddValue("totalTxPower",
                 "total tx power that will be proportionally assigned to"
                 " bandwidth parts depending on each BWP bandwidth ",
                 totalTxPower);
    cmd.AddValue("cellScan",
                 "Use beam search method to determine beamforming vector,"
                 "true to use cell scanning method",
                 cellScan);
    cmd.AddValue("beamSearchAngleStep",
                 "Beam search angle step for beam search method",
                 beamSearchAngleStep);
    cmd.AddValue("udpFullBuffer",
                 "Whether to set the full buffer traffic; if this parameter is "
                 "set then the udpInterval parameter will be neglected.",
                 udpFullBuffer);
    cmd.AddValue("logging", "Enable logging", logging);
    cmd.AddValue("simTag",
                 "tag to be appended to output filenames to distinguish simulation campaigns",
                 simTag);
    cmd.AddValue("outputDir", "directory where to store simulation results", outputDir);

    cmd.Parse(argc, argv);

//    NS_ABORT_MSG_IF(true, "Abort anyways");

    // ConfigStore inputConfig;
    // inputConfig.ConfigureDefaults ();

    // enable logging or not
    if (logging)
    {
//        LogComponentEnable("Nr3gppPropagationLossModel", LOG_LEVEL_ALL);
//        LogComponentEnable("Nr3gppBuildingsPropagationLossModel", LOG_LEVEL_ALL);
//        LogComponentEnable("Nr3gppChannel", LOG_LEVEL_ALL);
        LogComponentEnable("UdpClient", LOG_LEVEL_INFO);
        LogComponentEnable("UdpServer", LOG_LEVEL_INFO);
        LogComponentEnable("LtePdcp", LOG_LEVEL_INFO);
    }

    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(999999999));

    // create base stations and mobile terminals
    NodeContainer gNbNodes;
    NodeContainer ueNodes;
    MobilityHelper mobility;

    double gNbHeight = 10;
    double ueHeight = 1.5;

    gNbNodes.Create(1);
    ueNodes.Create(ueNumPergNb[0] + ueNumPergNb[1] + ueNumPergNb[2]);

    Ptr<ListPositionAllocator> apPositionAlloc = CreateObject<ListPositionAllocator>();
    Ptr<ListPositionAllocator> staPositionAlloc = CreateObject<ListPositionAllocator>();
    int32_t yValue = 0.0;

    for (uint32_t i = 1; i <= gNbNodes.GetN(); ++i)
    {
        // 2.0, -2.0, 6.0, -6.0, 10.0, -10.0, ....
        if (i % 2 != 0)
        {
            yValue = static_cast<int>(i) * 30;
        }
        else
        {
            yValue = -yValue;
        }

        apPositionAlloc->Add(Vector(0.0, yValue, gNbHeight));

        // 1.0, -1.0, 3.0, -3.0, 5.0, -5.0, ...
        double xValue = 0.0;
        for (uint32_t j = 1; j <= ueNumPergNb[0] + ueNumPergNb[1] + ueNumPergNb[2]; ++j)
        {
            if (j % 2 != 0)
            {
                xValue = j;
            }
            else
            {
                xValue = -xValue;
            }

            if (yValue > 0)
            {
                staPositionAlloc->Add(Vector(xValue, 10, ueHeight));
            }
            else
            {
                staPositionAlloc->Add(Vector(xValue, -10, ueHeight));
            }
        }
    }

    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(apPositionAlloc);
    mobility.Install(gNbNodes);

    mobility.SetPositionAllocator(staPositionAlloc);
    mobility.Install(ueNodes);

    // setup the nr simulation
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();

    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(epcHelper);

    /*
     * Setup the configuration of the spectrum. There is a contiguous and a non-contiguous
     * example:
     * 1) One operation band is deployed with 4 contiguous component carriers
     *    (CC)s, which are automatically generated by the ccBwpManager
     * 2) One operation bands non-contiguous case. CCs and BWPs are manually created
     */

    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;

    OperationBandInfo band;

    /**
    * Test: is arbitrary CC/BWP configuration supported?
    *
    * ----------------------------- Band --------------------------------
    * ------CC0------|--------CC1---------|-------------CC2--------------
    * ------BWP0-----|--------BWP1--------|-------------BWP2-------------
    */
    band.m_centralFrequency = centralFrequencyBand;
    band.m_channelBandwidth = bandwidthBand;
    band.m_lowerFrequency = band.m_centralFrequency - band.m_channelBandwidth / 2;
    band.m_higherFrequency = band.m_centralFrequency + band.m_channelBandwidth / 2;

    uint8_t bwpCount = 0;

    for (int n = 0; n < numCcs; ++n) {
        std::unique_ptr<ComponentCarrierInfo> cc0(new ComponentCarrierInfo());
        std::unique_ptr<BandwidthPartInfo> bwp0(new BandwidthPartInfo());

        // Component Carrier n
        cc0->m_ccId = n;
        cc0->m_centralFrequency = centralFrequencyCc[n];
        cc0->m_channelBandwidth = bandwidthCc[n];
        cc0->m_lowerFrequency = cc0->m_centralFrequency - cc0->m_channelBandwidth / 2;
        cc0->m_higherFrequency = cc0->m_centralFrequency + cc0->m_channelBandwidth / 2;

        // BWP n
        bwp0->m_bwpId = n;
        bwp0->m_centralFrequency = cc0->m_centralFrequency;
        bwp0->m_channelBandwidth = cc0->m_channelBandwidth;
        bwp0->m_lowerFrequency = cc0->m_lowerFrequency;
        bwp0->m_higherFrequency = cc0->m_higherFrequency;

        cc0->AddBwp(std::move(bwp0));
        ++bwpCount;

        // Add CC to the corresponding operation band.
        band.AddCc(std::move(cc0));
    }

    /**
     * Say something...
     */
    nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    epcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR"));
    // Beamforming method
    if (cellScan)
    {
        idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                             TypeIdValue(CellScanBeamforming::GetTypeId()));
        idealBeamformingHelper->SetBeamformingAlgorithmAttribute("BeamSearchAngleStep",
                                                                 DoubleValue(beamSearchAngleStep));
    }
    else
    {
        idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                             TypeIdValue(DirectPathBeamforming::GetTypeId()));
    }

    nrHelper->InitializeOperationBand(&band);
    allBwps = CcBwpCreator::GetAllBwps({band});

    double x = pow(10, totalTxPower / 10);

    // Antennas for all the UEs
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));

    // Antennas for all the gNbs
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<IsotropicAntennaModel>()));

    // VR, Cloud Gaming [CG], Autonomous Driving [AD]
    uint32_t bwpIdForVR = 0;
    uint32_t bwpIdForCG = 1;
    uint32_t bwpIdForAD = 2;

    nrHelper->SetGnbBwpManagerAlgorithmAttribute("GBR_GAMING",
                                                 UintegerValue(bwpIdForVR));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_VOICE_VIDEO_GAMING", UintegerValue(bwpIdForCG));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("GBR_V2X",
                                                 UintegerValue(bwpIdForAD));

    // Install and get the pointers to the NetDevices
    NetDeviceContainer enbNetDev = nrHelper->InstallGnbDevice(gNbNodes, allBwps);
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(ueNodes, allBwps);

    int64_t randomStream = 1;
    randomStream += nrHelper->AssignStreams(enbNetDev, randomStream);
    randomStream += nrHelper->AssignStreams(ueNetDev, randomStream);

    // Set the attribute of the netdevice (enbNetDev.Get (0)) and bandwidth part (0), (1), ...
    for (int n = 0; n < numCcs; ++n) {
        nrHelper->GetGnbPhy(enbNetDev.Get(0), n)
                ->SetAttribute("Numerology", UintegerValue(numerologyCc[n]));
        nrHelper->GetGnbPhy(enbNetDev.Get(0), n)
                ->SetAttribute(
                    "TxPower",
                    DoubleValue(10 *
                                log10((band.GetBwpAt(n, 0)->m_channelBandwidth / bandwidthBand) * x)));
        nrHelper->GetGnbPhy(enbNetDev.Get(0), 0)->SetAttribute("Pattern", StringValue(pattern));
    }

    for (auto it = enbNetDev.Begin(); it != enbNetDev.End(); ++it)
    {
        DynamicCast<NrGnbNetDevice>(*it)->UpdateConfig();
    }

    for (auto it = ueNetDev.Begin(); it != ueNetDev.End(); ++it)
    {
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
    }

    // create the internet and install the IP stack on the UEs
    // get SGW/PGW and create a single RemoteHost
    Ptr<Node> pgw = epcHelper->GetPgwNode();
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    // connect a remoteHost to pgw. Setup routing too
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(2500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.000)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIface;
    ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueNetDev));

    Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress(1);

    // Set the default gateway for the UEs
    for (uint32_t j = 0; j < ueNodes.GetN(); ++j)
    {
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(j)->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    // attach UEs to the closest eNB before creating the dedicated flows
    nrHelper->AttachToClosestEnb(ueNetDev, enbNetDev);

    // install UDP applications
    uint16_t dlPort = 1234;
    uint16_t ulPort = dlPort + (ueNumPergNb[0]+ueNumPergNb[1]+ueNumPergNb[2]) * numFlowsUe + 1;
    ApplicationContainer clientVrApps, clientCgApps, clientAdApps;
    ApplicationContainer serverApps;

    for (uint32_t u = 0; u < ueNumPergNb[0]; ++u)
    {
        // DL ONLY
        PacketSinkHelper dlPacketSinkHelper(
            "ns3::UdpSocketFactory",
            InetSocketAddress(Ipv4Address::GetAny(), dlPort));
        serverApps.Add(dlPacketSinkHelper.Install(ueNodes.Get(u)));

        // VR: 
        UdpClientHelper dlClient(ueIpIface.GetAddress(u), dlPort);
        dlClient.SetAttribute("PacketSize", UintegerValue(udpPacketSize[0]));
        dlClient.SetAttribute("Interval", TimeValue(Seconds(1.0 / lambda[0])));
        dlClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
        dlClient.SetAttribute("StartTime", TimeValue(Seconds(0.3+0.1*u)));
        dlClient.SetAttribute("StopTime", TimeValue(Seconds(0.4+0.1*u)));
        clientVrApps.Add(dlClient.Install(remoteHost));

        Ptr<EpcTft> tft = Create<EpcTft>();
        EpcTft::PacketFilter dlpf;
        dlpf.localPortStart = dlPort;
        dlpf.localPortEnd = dlPort;
        ++dlPort;
        tft->Add(dlpf);

        EpsBearer bearer(EpsBearer::GBR_GAMING);
        nrHelper->ActivateDedicatedEpsBearer(ueNetDev.Get(u), bearer, tft);
    }

    for (uint32_t u = ueNumPergNb[0]; u < ueNumPergNb[0] + ueNumPergNb[1]; ++u)
    {
        PacketSinkHelper dlPacketSinkHelper(
            "ns3::UdpSocketFactory",
            InetSocketAddress(Ipv4Address::GetAny(), dlPort));
        serverApps.Add(dlPacketSinkHelper.Install(ueNodes.Get(u)));

        // Cloud Gaming: 
        UdpClientHelper dlClient(ueIpIface.GetAddress(u), dlPort);
        dlClient.SetAttribute("PacketSize", UintegerValue(udpPacketSize[1]));
        dlClient.SetAttribute("Interval", TimeValue(Seconds(1.0 / lambda[1])));
        dlClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
        clientCgApps.Add(dlClient.Install(remoteHost));

        Ptr<EpcTft> tft = Create<EpcTft>();
        EpcTft::PacketFilter dlpf;
        dlpf.localPortStart = dlPort;
        dlpf.localPortEnd = dlPort;
        ++dlPort;
        tft->Add(dlpf);

        EpsBearer bearer(EpsBearer::NGBR_VOICE_VIDEO_GAMING);
        nrHelper->ActivateDedicatedEpsBearer(ueNetDev.Get(u), bearer, tft);
    }

    for (uint32_t u = ueNumPergNb[0] + ueNumPergNb[1]; u < ueNodes.GetN(); ++u)
    {
        // DL ONLY
        PacketSinkHelper dlPacketSinkHelper(
            "ns3::UdpSocketFactory",
            InetSocketAddress(Ipv4Address::GetAny(), dlPort));
        serverApps.Add(dlPacketSinkHelper.Install(ueNodes.Get(u)));

        UdpClientHelper dlClient(ueIpIface.GetAddress(u), dlPort);
        dlClient.SetAttribute("PacketSize", UintegerValue(udpPacketSize[2]));
        dlClient.SetAttribute("Interval", TimeValue(Seconds(1.0 / lambda[2])));
        dlClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
        clientAdApps.Add(dlClient.Install(remoteHost));

        Ptr<EpcTft> tft = Create<EpcTft>();
        EpcTft::PacketFilter dlpf;
        dlpf.localPortStart = dlPort;
        dlpf.localPortEnd = dlPort;
        ++dlPort;
        tft->Add(dlpf);
        
        EpsBearer bearer(EpsBearer::NGBR_VIDEO_TCP_DEFAULT);
        nrHelper->ActivateDedicatedEpsBearer(ueNetDev.Get(u), bearer, tft);
    }

    // start UDP server and client apps
    serverApps.Start(Seconds(udpAppStartTime));
    clientCgApps.Start(Seconds(udpAppStartTime));
    clientAdApps.Start(Seconds(udpAppStartTime));
    serverApps.Stop(Seconds(simTime));
    clientCgApps.Stop(Seconds(simTime));
    clientAdApps.Stop(Seconds(simTime));

    // enable the traces provided by the nr module
    nrHelper->EnableTraces();

    FlowMonitorHelper flowmonHelper;
    NodeContainer endpointNodes;
    endpointNodes.Add(remoteHost);
    endpointNodes.Add(ueNodes);

    Ptr<ns3::FlowMonitor> monitor = flowmonHelper.Install(endpointNodes);
    monitor->SetAttribute("DelayBinWidth", DoubleValue(0.001));
    monitor->SetAttribute("JitterBinWidth", DoubleValue(0.001));
    monitor->SetAttribute("PacketSizeBinWidth", DoubleValue(20));

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    /*
     * To check what was installed in the memory, i.e., BWPs of eNb Device, and its configuration.
     * Example is: Node 1 -> Device 0 -> BandwidthPartMap -> {0,1} BWPs -> NrGnbPhy ->
    NrPhyMacCommong-> Numerology, Bandwidth, ... GtkConfigStore config; config.ConfigureAttributes
    ();
    */

    // simulation finished...
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "RUNTIME: " << elapsed_seconds.count() << "s" << std::endl;

    // Print per-flow statistics
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();

    double averageFlowThroughput = 0.0;
    double averageFlowDelay = 0.0;

    std::ofstream outFile;
    std::string filename = outputDir + "/" + simTag;
    outFile.open(filename.c_str(), std::ofstream::out | std::ofstream::trunc);
    if (!outFile.is_open())
    {
        std::cerr << "Can't open file " << filename << std::endl;
        return 1;
    }

    outFile.setf(std::ios_base::fixed);

    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin();
         i != stats.end();
         ++i)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);
        std::stringstream protoStream;
        protoStream << (uint16_t)t.protocol;
        if (t.protocol == 6)
        {
            protoStream.str("TCP");
        }
        if (t.protocol == 17)
        {
            protoStream.str("UDP");
        }
        outFile << "Flow " << i->first << " (" << t.sourceAddress << ":" << t.sourcePort << " -> "
                << t.destinationAddress << ":" << t.destinationPort << ") proto "
                << protoStream.str() << "\n";
        outFile << "  Tx Packets: " << i->second.txPackets << "\n";
        outFile << "  Tx Bytes:   " << i->second.txBytes << "\n";
        outFile << "  TxOffered:  "
                << i->second.txBytes * 8.0 / (simTime - udpAppStartTime) / 1000 / 1000 << " Mbps\n";
        outFile << "  Rx Bytes:   " << i->second.rxBytes << "\n";
        if (i->second.rxPackets > 0)
        {
            // Measure the duration of the flow from receiver's perspective
            // double rxDuration = i->second.timeLastRxPacket.GetSeconds () -
            // i->second.timeFirstTxPacket.GetSeconds ();
            double rxDuration = i->second.timeLastRxPacket.GetSeconds () -
                i->second.timeFirstTxPacket.GetSeconds ();

            averageFlowThroughput += i->second.rxBytes * 8.0 / rxDuration / 1000 / 1000;
            averageFlowDelay += 1000 * i->second.delaySum.GetSeconds() / i->second.rxPackets;

            outFile << "  Throughput: " << i->second.rxBytes * 8.0 / rxDuration / 1000 / 1000
                    << " Mbps\n";
            outFile << "  Mean delay:  "
                    << 1000 * i->second.delaySum.GetSeconds() / i->second.rxPackets << " ms\n";
            // outFile << "  Mean upt:  " << i->second.uptSum / i->second.rxPackets / 1000/1000 << "
            // Mbps \n";
            outFile << "  Mean jitter:  "
                    << 1000 * i->second.jitterSum.GetSeconds() / i->second.rxPackets << " ms\n";
        }
        else
        {
            outFile << "  Throughput:  0 Mbps\n";
            outFile << "  Mean delay:  0 ms\n";
            outFile << "  Mean jitter: 0 ms\n";
        }
        outFile << "  Rx Packets: " << i->second.rxPackets << "\n";
    }

    outFile << "\n\n  Mean flow throughput: " << averageFlowThroughput / stats.size() << "\n";
    outFile << "  Mean flow delay: " << averageFlowDelay / stats.size() << "\n";

    outFile.close();

    std::ifstream f(filename.c_str());

    if (f.is_open())
    {
        std::cout << f.rdbuf();
    }

    Simulator::Destroy();
    return 0;
}

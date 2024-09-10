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

void
ConfigureXrApp(NodeContainer& ueContainer,
               uint32_t i,
               Ipv4InterfaceContainer& ueIpIface,
               enum NrXrConfig config,
               double appDataRate,
               uint16_t appFps,
               uint16_t port,
               std::string transportProtocol,
               NodeContainer& remoteHostContainer,
               NetDeviceContainer& ueNetDev,
               Ptr<NrHelper> nrHelper,
               EpsBearer& bearer,
               Ptr<EpcTft> tft,
               bool isMx1,
               std::vector<Ptr<EpcTft>>& tfts,
               ApplicationContainer& serverApps,
               ApplicationContainer& clientApps,
               ApplicationContainer& pingApps)
{
    XrTrafficMixerHelper trafficMixerHelper;
    Ipv4Address ipAddress = ueIpIface.GetAddress(i, 0);
    trafficMixerHelper.ConfigureXr(config);
    auto it = XrPreconfig.find(config);

    std::vector<Address> addresses;
    std::vector<InetSocketAddress> localAddresses;
    for (size_t j = 0; j < it->second.size(); j++)
    {
        addresses.emplace_back(InetSocketAddress(ipAddress, port + j));
        // The sink will always listen to the specified ports
        localAddresses.emplace_back(Ipv4Address::GetAny(), port + j);
    }

    ApplicationContainer currentUeClientApps;
    currentUeClientApps.Add(
        trafficMixerHelper.Install(transportProtocol, addresses, remoteHostContainer.Get(0)));

    // Seed the ARP cache by pinging early in the simulation
    // This is a workaround until a static ARP capability is provided
    PingHelper ping(ipAddress);
    pingApps.Add(ping.Install(remoteHostContainer));

    Ptr<NetDevice> ueDevice = ueNetDev.Get(i);
    // Activate a dedicated bearer for the traffic type per node
    nrHelper->ActivateDedicatedEpsBearer(ueDevice, bearer, tft);
    // Activate a dedicated bearer for the traffic type per node
    if (isMx1)
    {
        nrHelper->ActivateDedicatedEpsBearer(ueDevice, bearer, tft);
    }
    else
    {
        NS_ASSERT(tfts.size() >= currentUeClientApps.GetN());
        for (uint32_t j = 0; j < currentUeClientApps.GetN(); j++)
        {
            nrHelper->ActivateDedicatedEpsBearer(ueDevice, bearer, tfts[j]);
        }
    }

    for (uint32_t j = 0; j < currentUeClientApps.GetN(); j++)
    {
        PacketSinkHelper dlPacketSinkHelper(transportProtocol, localAddresses.at(j));
        Ptr<Application> packetSink = dlPacketSinkHelper.Install(ueContainer.Get(i)).Get(0);
        serverApps.Add(packetSink);
        Ptr<TrafficGenerator3gppGenericVideo> app =
            DynamicCast<TrafficGenerator3gppGenericVideo>(currentUeClientApps.Get(j));
        if (app)
        {
            app->SetAttribute("DataRate", DoubleValue(appDataRate));
            app->SetAttribute("Fps", UintegerValue(appFps));
        }
    }
    clientApps.Add(currentUeClientApps);
}

int
main(int argc, char* argv[])
{
    uint32_t appDuration = 10000;
    uint32_t appStartTimeMs = 400;

    const uint8_t numCcs = 3;

    uint16_t ueNumPerSlice [] = {1, 2, 3};
    uint16_t numFlowsUe = 1;

    // 5G NR n256 (FR2)
    double bandwidthBand = 3e9;
    double centralFrequencyBand = 28e9;

    // general(non-contiguous) cc setting
    double bandwidthCc [] = {2e9, 0.5e9, 0.5e9};
    double centralFrequencyCc [] = {
        centralFrequencyBand - bandwidthCc[1] / 2. - bandwidthCc[2] / 2.,
        centralFrequencyBand,
        centralFrequencyBand +  bandwidthCc[0] / 2. + bandwidthCc[1] / 2.
    };
    uint16_t numerologyCc [] = {3, 3, 3};


    std::string pattern =
        "DL|DL|DL|DL|UL|DL|DL|DL|DL|UL|"; // Pattern can be e.g. "DL|S|UL|UL|DL|DL|S|UL|UL|DL|" "F|F|F|F|F|F|F|F|F|F|"
    double totalTxPower = 41;
    bool cellScan = false;
    double beamSearchAngleStep = 10.0;

    bool useUdp = false;
    double dataRate [] = {45., 30., 10.};   // data rate in Mbps
    uint16_t fps [] = {60, 60, 30};
    bool logging = false;

    // commencing...
    auto start = std::chrono::system_clock::now();
    std::time_t start_time = std::chrono::system_clock::to_time_t(start);
    std::string simTag = std::ctime(&start_time);
    std::string outputDir = "./";

    // random seed
    uint32_t rngRun = 1;

    CommandLine cmd(__FILE__);

    cmd.AddValue("appDuration", "Duration of the application in milliseconds.", appDuration);
    cmd.AddValue("ueNumPerSlice0", "The number of UE of VR in multiple-ue topology", ueNumPerSlice[0]);
    cmd.AddValue("ueNumPerSlice1", "The number of UE of CG in multiple-ue topology", ueNumPerSlice[1]);
    cmd.AddValue("ueNumPerSlice2", "The number of UE of AD in multiple-ue topology", ueNumPerSlice[2]);
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
    cmd.AddValue("useUdp",
                 "if true, the NGMN applications will run over UDP connection, otherwise a TCP "
                 "connection will be used.",
                 useUdp);
    cmd.AddValue("rngRun", "Rng run random number.", rngRun);
    cmd.AddValue("logging", "Enable logging", logging);
    cmd.AddValue("simTag",
                 "tag to be appended to output filenames to distinguish simulation campaigns",
                 simTag);
    cmd.AddValue("outputDir", "directory where to store simulation results", outputDir);

    cmd.Parse(argc, argv);

//    NS_ABORT_MSG_IF(true, "Abort anyways");

    // ConfigStore inputConfig;
    // inputConfig.ConfigureDefaults ();

    // Set simulation run number
    SeedManager::SetRun(rngRun);

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

    double gNbHeight = 25;
    double ueHeight = 1.5;

    gNbNodes.Create(1);
    ueNodes.Create(ueNumPerSlice[0] + ueNumPerSlice[1] + ueNumPerSlice[2]);

    Ptr<ListPositionAllocator> bsPositionAlloc = CreateObject<ListPositionAllocator>();
    bsPositionAlloc->Add(Vector(0.0, 0.0, gNbHeight));
    mobility.SetPositionAllocator(bsPositionAlloc);
    mobility.Install(gNbNodes);

    Ptr<RandomDiscPositionAllocator> ueDiscPositionAlloc =
        CreateObject<RandomDiscPositionAllocator>();
    ueDiscPositionAlloc->SetX(0.0);
    ueDiscPositionAlloc->SetY(0.0);
    ueDiscPositionAlloc->SetZ(ueHeight);
    mobility.SetPositionAllocator(ueDiscPositionAlloc);
    for (uint32_t i = 0; i < ueNodes.GetN(); i++)
    {
        mobility.Install(ueNodes.Get(i));
    }

    /** 
     * NR Simulation Setup;
     */
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();

    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(epcHelper);

    /**
     * Bandwidth Part Setup; 1 CC = 1 BWP, Arbitrary BW 
     *
     * ----------------------------- Band --------------------------------
     * ------CC0------|--------CC1---------|-------------CC2--------------
     * ------BWP0-----|--------BWP1--------|-------------BWP2-------------
     */
    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;

    OperationBandInfo band;

    band.m_centralFrequency = centralFrequencyBand;
    band.m_channelBandwidth = bandwidthBand;
    band.m_lowerFrequency = band.m_centralFrequency - band.m_channelBandwidth / 2;
    band.m_higherFrequency = band.m_centralFrequency + band.m_channelBandwidth / 2;

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
        band.AddCc(std::move(cc0));
    }

    /**
     * Beamforming Model Setup;
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

    nrHelper->SetGnbPhyAttribute("NoiseFigure", DoubleValue(5));
    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(23));
    nrHelper->SetUePhyAttribute("NoiseFigure", DoubleValue(7));

    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(999999999));
    Config::SetDefault("ns3::LteEnbRrc::EpsBearerToRlcMapping",
                       EnumValue(useUdp ? LteEnbRrc::RLC_UM_ALWAYS : LteEnbRrc::RLC_AM_ALWAYS));

    /**
     * Antenna Model Setup;
     */
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<ThreeGppAntennaModel>()));
    nrHelper->SetGnbAntennaAttribute("AntennaHorizontalSpacing", DoubleValue(0.5));
    nrHelper->SetGnbAntennaAttribute("AntennaVerticalSpacing", DoubleValue(0.8));
    nrHelper->SetGnbAntennaAttribute("DowntiltAngle", DoubleValue(0 * M_PI / 180.0));
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));   // 2?
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(1));// 4?
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));

    // VR, Cloud Gaming [CG], Autonomous Driving [AD]
    uint32_t bwpIdForVR = 0;
    uint32_t bwpIdForCG = 1;
    uint32_t bwpIdForAD = 2;

    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_VIDEO_TCP_DEFAULT",
                                                 UintegerValue(bwpIdForVR));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_VOICE_VIDEO_GAMING", UintegerValue(bwpIdForCG));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_V2X",
                                                 UintegerValue(bwpIdForAD));

    // Install and get the pointers to the NetDevices
    NodeContainer ueVrNodes, ueCgNodes, ueAdNodes; 
    for (auto j = 0; j < ueNumPerSlice[0]; ++j)
    {
        Ptr<Node> ue = ueNodes.Get(j);
        ueVrNodes.Add(ue);
    }
    for (auto j = ueNumPerSlice[0]; j < ueNumPerSlice[0] + ueNumPerSlice[1]; ++j)
    {
        Ptr<Node> ue = ueNodes.Get(j);
        ueCgNodes.Add(ue);
    }
    for (auto j = ueNumPerSlice[0] + ueNumPerSlice[1]; j < ueNumPerSlice[0] + ueNumPerSlice[1] + ueNumPerSlice[2]; ++j)
    {
        Ptr<Node> ue = ueNodes.Get(j);
        ueAdNodes.Add(ue);
    }

    NetDeviceContainer gNbNetDev = nrHelper->InstallGnbDevice(gNbNodes, allBwps);
    NetDeviceContainer ueVrNetDev = nrHelper->InstallUeDevice(ueVrNodes, allBwps);
    NetDeviceContainer ueCgNetDev = nrHelper->InstallUeDevice(ueCgNodes, allBwps);
    NetDeviceContainer ueAdNetDev = nrHelper->InstallUeDevice(ueAdNodes, allBwps);

    int64_t randomStream = 1;
    randomStream += nrHelper->AssignStreams(gNbNetDev, randomStream);
    randomStream += nrHelper->AssignStreams(ueVrNetDev, randomStream);
    randomStream += nrHelper->AssignStreams(ueCgNetDev, randomStream);
    randomStream += nrHelper->AssignStreams(ueAdNetDev, randomStream);

    // Set the attribute of the netdevice (gNbNetDev.Get (0)) and bandwidth part (0), (1), ...
    double x = pow(10, totalTxPower / 10);
    for (int n = 0; n < numCcs; ++n) {
        nrHelper->GetGnbPhy(gNbNetDev.Get(0), n)
                ->SetAttribute("Numerology", UintegerValue(numerologyCc[n]));
        nrHelper->GetGnbPhy(gNbNetDev.Get(0), n)
                ->SetAttribute(
                    "TxPower",
                    DoubleValue(10 *
                                log10((band.GetBwpAt(n, 0)->m_channelBandwidth / bandwidthBand) * x)));
        nrHelper->GetGnbPhy(gNbNetDev.Get(0), n)->SetAttribute("Pattern", StringValue(pattern));
    }

    for (auto it = gNbNetDev.Begin(); it != gNbNetDev.End(); ++it)
    {
        DynamicCast<NrGnbNetDevice>(*it)->UpdateConfig();
    }

    for (auto it = ueVrNetDev.Begin(); it != ueVrNetDev.End(); ++it)
    {
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
    }

    for (auto it = ueCgNetDev.Begin(); it != ueCgNetDev.End(); ++it)
    {
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
    }

    for (auto it = ueAdNetDev.Begin(); it != ueAdNetDev.End(); ++it)
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

    Ipv4InterfaceContainer ueVrIpIface, ueCgIpIface, ueAdIpIface;
    ueVrIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueVrNetDev));
    ueCgIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueCgNetDev));
    ueAdIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueAdNetDev));

    // Set the default gateway for the UEs
    for (uint32_t j = 0; j < ueNodes.GetN(); ++j)
    {
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(j)->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    // attach UEs to the closest eNB before creating the dedicated flows
    nrHelper->AttachToClosestEnb(ueVrNetDev, gNbNetDev);
    nrHelper->AttachToClosestEnb(ueCgNetDev, gNbNetDev);
    nrHelper->AttachToClosestEnb(ueAdNetDev, gNbNetDev);

    // install generic 3GPP video applications
    std::string transportProtocol = useUdp ?
        "ns3::UdpSocketFactory" : "ns3::TcpSocketFactory";
    uint16_t dlVrPort = 1001;
    uint16_t dlCgPort = 1101;
    uint16_t dlAdPort = 1201;
    ApplicationContainer clientVrApps, clientCgApps, clientAdApps;
    ApplicationContainer serverApps, pingApps;

    EpsBearer vrBearer(EpsBearer::NGBR_VIDEO_TCP_DEFAULT);
    Ptr<EpcTft> vrTft = Create<EpcTft>();
    EpcTft::PacketFilter dlpfVr;
    dlpfVr.localPortStart = dlVrPort;
    dlpfVr.localPortEnd = dlVrPort;
    vrTft->Add(dlpfVr);

    EpsBearer cgBearer(EpsBearer::NGBR_VOICE_VIDEO_GAMING);
    Ptr<EpcTft> cgTft = Create<EpcTft>();
    EpcTft::PacketFilter dlpfCg;
    dlpfCg.localPortStart = dlCgPort;
    dlpfCg.localPortEnd = dlCgPort;
    cgTft->Add(dlpfCg);

    EpsBearer adBearer(EpsBearer::NGBR_V2X);
    Ptr<EpcTft> adTft = Create<EpcTft>();
    EpcTft::PacketFilter dlpfAd;
    dlpfAd.localPortStart = dlAdPort;
    dlpfAd.localPortEnd = dlAdPort;
    adTft->Add(dlpfAd);

    std::vector<Ptr<EpcTft>> arTfts;    // unused;

    for (uint32_t u = 0; u < ueNumPerSlice[0]; ++u)
    {
        ConfigureXrApp(
            ueVrNodes,
            u,
            ueVrIpIface,
            VR_DL1,//AR_M3,
            dataRate[0],
            fps[0],
            dlVrPort,
            transportProtocol,
            remoteHostContainer,
            ueVrNetDev,
            nrHelper,
            vrBearer,
            vrTft,
            true,
            arTfts,
            serverApps,
            clientVrApps,
            pingApps);
    }

    for (uint32_t u = ueNumPerSlice[0]; u < ueNumPerSlice[0] + ueNumPerSlice[1]; ++u)
    {        
        ConfigureXrApp(
            ueCgNodes,
            u - ueNumPerSlice[0],
            ueCgIpIface,
            VR_DL1,//AR_M3,
            dataRate[1],
            fps[1],
            dlCgPort,
            transportProtocol,
            remoteHostContainer,
            ueCgNetDev,
            nrHelper,
            cgBearer,
            cgTft,
            true,
            arTfts,
            serverApps,
            clientCgApps,
            pingApps
        );
    }

    for (uint32_t u = ueNumPerSlice[0] + ueNumPerSlice[1]; u < ueNodes.GetN(); ++u)
    {
        ConfigureXrApp(
            ueAdNodes,
            u - ueNumPerSlice[0] - ueNumPerSlice[1],
            ueAdIpIface,
            VR_DL1,//AR_M3,
            dataRate[2],
            fps[2],
            dlAdPort,
            transportProtocol,
            remoteHostContainer,
            ueAdNetDev,
            nrHelper,
            adBearer,
            adTft,
            true,
            arTfts,
            serverApps,
            clientAdApps,
            pingApps
        );
    }

    // start UDP server and client apps
    pingApps.Start(MilliSeconds(100));
    pingApps.Stop(MilliSeconds(appStartTimeMs));

    // start server and client apps
    uint32_t simTimeMs = appStartTimeMs + appDuration + 2000;

    serverApps.Start(MilliSeconds(appStartTimeMs));
    clientVrApps.Start(MilliSeconds(appStartTimeMs));
    clientCgApps.Start(MilliSeconds(appStartTimeMs));
    clientAdApps.Start(MilliSeconds(appStartTimeMs));
    serverApps.Stop(MilliSeconds(simTimeMs));
    clientVrApps.Stop(MilliSeconds(appStartTimeMs + appDuration));
    clientCgApps.Stop(MilliSeconds(appStartTimeMs + appDuration));
    clientAdApps.Stop(MilliSeconds(appStartTimeMs + appDuration));

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

    Simulator::Stop(MilliSeconds(simTimeMs));
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
        double txDuration = MilliSeconds(appDuration).GetSeconds();
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
                << i->second.txBytes * 8.0 / txDuration / 1000 / 1000 << " Mbps\n";
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

/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 *   Copyright (c) 2021 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License version 2 as
 *   published by the Free Software Foundation;
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/buildings-module.h"
#include "ns3/config-store-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("SpatialMultiplexingForNRUCoexintence");

int
main(int argc, char* argv[])
{
	// Scenario parameters
	// Basic configuration for gNB
	uint16_t gNbNum = 1;
    uint16_t numRowsGnb = 2;
    uint16_t numColumnsGnb = 2;
    // Basic configuration for UE/s
    uint16_t ueNumNR = 1;
    uint16_t numRowsUe = 1;
    uint16_t numColumnsUe = 1;
	uint32_t udpPacketSize = 1000;				// Size of UDP packet
    Time packetInterval = NanoSeconds(40000);	// For 2x2 MIMO and NR MCS table 2, packet interval is 40000 ns to reach 200 mb/s TODO [change this value]

	// Simulation times
    Time simTime = MilliSeconds(1000);
    Time udpAppStartTime = MilliSeconds(400);

    // NR slice parameters
    uint16_t numerology = 0;
    double centralFrequency = 5.8e9;		// Central frequency of ISM 5GHz band
    double bandwidth = 150e6;				// Bandwidth of this frequency band
    double gnbTxPower = 40; 				// Output of gNB antenna in dBm
    double ueTxPower = 23;  				// UE output power in dBm
    // Polaryzation of slant
    double polSlantAngle1 = 0.0;
    double polSlantAngle2 = 90.0;
    double polarizationFirstSubArray = (polSlantAngle1 * M_PI) / 180.0;  								// converting to radians
    double polarizationSecondSubArray = (polSlantAngle2 * M_PI) / 180.0; 								// converting to radians

    int64_t randomStream = 1;

    // Output of simulation
    std::string simTag = "default";
    std::string outputDir = "./";

    // Informing ns3 of input simulation parameters
    CommandLine cmd(__FILE__);

    cmd.AddValue("gNbNum", "The number of gNb in topology", gNbNum);
    cmd.AddValue("numRowsGnb", "Number of antenna rows at the gNB", numRowsGnb);
    cmd.AddValue("numColumnsGnb", "Number of antenna columns at the gNB", numColumnsGnb);
    cmd.AddValue("ueNumNR", "The number of cellular UEs in topology", ueNumNR);
    cmd.AddValue("numRowsUe", "Number of antenna rows at the UE", numRowsUe);
    cmd.AddValue("numColumnsUe", "Number of antenna columns at the UE", numColumnsUe);
    cmd.AddValue("packetSize", "Packet size in bytes", udpPacketSize);
    cmd.AddValue("packetInterval", "Inter packet interval for CBR traffic", packetInterval);
    cmd.AddValue("simTime", "Simulation time", simTime);
    cmd.AddValue("numerology", "The numerology to be used", numerology);
    cmd.AddValue("centralFrequency", "The system frequency to be used for 5GHz ISM band", centralFrequency);
    cmd.AddValue("bandwidth", "The system bandwidth to be used", bandwidth);
    cmd.AddValue("gnbTxPower", "gNB TX power", gnbTxPower);
    cmd.AddValue("ueTxPower", "UE TX power", ueTxPower);
    cmd.AddValue("polSlantAngle1", "Polarization slant angle of the first antenna sub-array/partition in degrees", polSlantAngle1);
    cmd.AddValue("polSlantAngle2", "Polarization slant angle of the second antenna sub-array/partition in degrees", polSlantAngle2);
    cmd.AddValue("simTag", "Tag to be appended to output filenames to distinguish simulation campaigns", simTag);
    cmd.AddValue("outputDir", "Directory where to store simulation results", outputDir);

    cmd.Parse(argc, argv);					// parser to command line

    // Default values for the simulation. Needed for legacy (LTE) code
    // And setting attributes for ThreeGppChannelModel
    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(999999999));
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(0)));
    Config::SetDefault("ns3::ThreeGppSpectrumPropagationLossModel::ChannelModel", StringValue("ns3::ThreeGppChannelModelParam"));

    // Create node container for gNB and UE/s
    NodeContainer gnbContainer;
    gnbContainer.Create(1);
    NodeContainer ueContainer;
    ueContainer.Create(1);

    // Set mobility model as a constant position for both gNB and UE
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> positionAllocUe = CreateObject<ListPositionAllocator>();
    positionAllocUe->Add(Vector(0.0, 0.0, 10.0));		// gNB in position (0,0,0) with 10 meters high
    positionAllocUe->Add(Vector(20, 0.0, 1.5));			// UE in position (20,0,0) with 1.5 meter high
    mobility.SetPositionAllocator(positionAllocUe);		// the distance between node is constant and equal to 20 meters
    mobility.Install(gnbContainer);
    mobility.Install(ueContainer);

    // Setup NR by using helpers
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();				// setup the core network as EPC from LTE
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();	// takes care of the beamforming part
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();												// takes care of creating and connecting the NR stack
    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(epcHelper);

    // Spectrum configuration - single operational band and configure the scenario.
    BandwidthPartInfoPtrVector allBwps;																// contain all spectrum configuration
    CcBwpCreator ccBwpCreator;
    const uint8_t numCcPerBand = 1;

    CcBwpCreator::SimpleOperationBandConf bandConf(centralFrequency,								// create a configuration for band
                                                   bandwidth,
                                                   numCcPerBand,
                                                   BandwidthPartInfo::UMi_StreetCanyon);			// using UMi_StreetCanyon scenario

    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);				// create band by using band defined band configuration
    // Update channel settings
    nrHelper->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(MilliSeconds(0)));
    nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));

    nrHelper->InitializeOperationBand(&band);		// Initialized channel
    allBwps = CcBwpCreator::GetAllBwps({band});		// get spectrum configuration for the band

    Packet::EnableChecking();
    Packet::EnablePrinting();

    // Setting attributes valid for all nodes

    idealBeamformingHelper->SetAttribute("BeamformingMethod", TypeIdValue(DirectPathBeamforming::GetTypeId()));		// Beamforming method
    epcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));											// Core latency

    // Setting antennas for the gNB
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(numRowsGnb));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(numColumnsGnb));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",	PointerValue(CreateObject<IsotropicAntennaModel>()));		// Using isotropic antenna model for gNB
    // Setting antennas for the UE/s
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(ueNumNR));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(numRowsUe));
    nrHelper->SetUeAntennaAttribute("AntennaElement",PointerValue(CreateObject<IsotropicAntennaModel>()));			// Using isotropic antenna model for UE/s

    // Setting routing between Bearer and bandwidth part
    uint32_t bwpId = 0;
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(bwpId));	    // for gNb
    nrHelper->SetUeBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(bwpId));			// for UE

    // Installation of configured attributes and getting pointer to the NetDevices which contains all the NR stack:

    NetDeviceContainer enbNetDev = nrHelper->InstallGnbDevice(gnbContainer, allBwps);
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(ueContainer, allBwps);

    randomStream += nrHelper->AssignStreams(enbNetDev, randomStream);
    randomStream += nrHelper->AssignStreams(ueNetDev, randomStream);

    // Setting attributes for gNB NetDevice
    nrHelper->GetGnbPhy(enbNetDev.Get(0), 0)->SetAttribute("Numerology", UintegerValue(numerology));	// set numerology
    nrHelper->GetGnbPhy(enbNetDev.Get(0), 0)->SetAttribute("TxPower", DoubleValue(gnbTxPower));			// set TxPower
    ObjectVectorValue gnbSpectrumPhys;
    Ptr<NrSpectrumPhy> nrSpectrumPhy;
    nrHelper->GetGnbPhy(enbNetDev.Get(0), 0)->GetAttribute("NrSpectrumPhyList", gnbSpectrumPhys);
    nrSpectrumPhy = gnbSpectrumPhys.Get(0)->GetObject<NrSpectrumPhy>();
    nrSpectrumPhy->GetAntenna()->GetObject<UniformPlanarArray>()->SetAttribute("PolSlantAngle", DoubleValue(polarizationFirstSubArray));
    if (gnbSpectrumPhys.GetN() == 2)
    {
        nrSpectrumPhy = gnbSpectrumPhys.Get(1)->GetObject<NrSpectrumPhy>();
        nrSpectrumPhy->GetAntenna()->GetObject<UniformPlanarArray>()->SetAttribute("PolSlantAngle", DoubleValue(polarizationSecondSubArray));
    }

    // Setting attributes for UE/s NetDevice
    nrHelper->GetUePhy(ueNetDev.Get(0), 0)->SetAttribute("TxPower", DoubleValue(ueTxPower));			// set TxPower
    ObjectVectorValue ueSpectrumPhys;
    nrHelper->GetUePhy(ueNetDev.Get(0), 0)->GetAttribute("NrSpectrumPhyList", ueSpectrumPhys);
    nrSpectrumPhy = ueSpectrumPhys.Get(0)->GetObject<NrSpectrumPhy>();
    nrSpectrumPhy->GetAntenna()->GetObject<UniformPlanarArray>()->SetAttribute("PolSlantAngle", DoubleValue(polarizationFirstSubArray));
    if (ueSpectrumPhys.GetN() == 2)
    {
        nrSpectrumPhy = ueSpectrumPhys.Get(1)->GetObject<NrSpectrumPhy>();
        nrSpectrumPhy->GetAntenna()->GetObject<UniformPlanarArray>()->SetAttribute("PolSlantAngle", DoubleValue(polarizationSecondSubArray));
    }

    // When all the configuration is done, explicitly call UpdateConfig ()
    for (auto it = enbNetDev.Begin(); it != enbNetDev.End(); ++it) {DynamicCast<NrGnbNetDevice>(*it)->UpdateConfig();}
    for (auto it = ueNetDev.Begin(); it != ueNetDev.End(); ++it) {DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();}

    // NS3 configuration start here

    // create the Internet and install the IP stack on the UEs; get SGW/PGW and create a single RemoteHost
    Ptr<Node> pgw = epcHelper->GetPgwNode();
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    // connect a remoteHost to pgw. Setup routing too
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.000)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);
    internet.Install(ueContainer);
    Ipv4InterfaceContainer ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueNetDev));
    Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting(ueContainer.Get(0)->GetObject<Ipv4>());	// Set the default gateway for the UE
    ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);

    // attach UE to the closest eNB
    nrHelper->AttachToClosestEnb(ueNetDev, enbNetDev);

    // Traffic part

    uint16_t dlPort = 1234;								// choosing port
    ApplicationContainer serverApps;					// create server app for traffic listening
    UdpServerHelper dlPacketSink(dlPort);				// setting communication port
    serverApps.Add(dlPacketSink.Install(ueContainer));	// install server app on UE for listening

    // Configure attributes for the CBR traffic generator, using user-provided parameters
    UdpClientHelper dlClient;
    dlClient.SetAttribute("RemotePort", UintegerValue(dlPort));
    dlClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
    dlClient.SetAttribute("PacketSize", UintegerValue(udpPacketSize));
    dlClient.SetAttribute("Interval", TimeValue(packetInterval));

    EpsBearer epsBearer(EpsBearer::NGBR_LOW_LAT_EMBB);	// The bearer that will carry the traffic

    // The filter for the traffic
    Ptr<EpcTft> dlTft = Create<EpcTft>();
    EpcTft::PacketFilter dlPktFilter;
    dlPktFilter.localPortStart = dlPort;
    dlPktFilter.localPortEnd = dlPort;
    dlTft->Add(dlPktFilter);

    // Install client application for serving DL throughput
    ApplicationContainer clientApps;
    for (uint32_t i = 0; i < ueContainer.GetN(); ++i)
    {
        Ptr<Node> ue = ueContainer.Get(i);
        Ptr<NetDevice> ueDevice = ueNetDev.Get(i);
        Address ueAddress = ueIpIface.GetAddress(i);
        dlClient.SetAttribute("RemoteAddress", AddressValue(ueAddress));
        clientApps.Add(dlClient.Install(remoteHost));
        nrHelper->ActivateDedicatedEpsBearer(ueDevice, epsBearer, dlTft);
    }

    // start UDP server and client apps
    serverApps.Start(udpAppStartTime);
    clientApps.Start(udpAppStartTime);
    serverApps.Stop(simTime);
    clientApps.Stop(simTime);

    nrHelper->EnableTraces();	// enable the traces provided by the nr module

    FlowMonitorHelper flowmonHelper;
    NodeContainer endpointNodes;
    endpointNodes.Add(remoteHost);
    endpointNodes.Add(ueContainer);

    Ptr<ns3::FlowMonitor> monitor = flowmonHelper.Install(endpointNodes);
    monitor->SetAttribute("DelayBinWidth", DoubleValue(0.001));
    monitor->SetAttribute("JitterBinWidth", DoubleValue(0.001));
    monitor->SetAttribute("PacketSizeBinWidth", DoubleValue(20));

    Simulator::Stop(simTime);
    Simulator::Run();			// Start the simulation

    // Print per-flow statistics
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
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

    double flowDuration = (simTime - udpAppStartTime).GetSeconds();
    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin(); i != stats.end(); ++i)
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
                << t.destinationAddress << ":" << t.destinationPort << ") proto " << protoStream.str() << "\n";
        outFile << "  Tx Packets: " << i->second.txPackets << "\n";
        outFile << "  Tx Bytes:   " << i->second.txBytes << "\n";
        outFile << "  TxOffered:  " << i->second.txBytes * 8.0 / flowDuration / 1000.0 / 1000.0 << " Mbps\n";
        outFile << "  Rx Bytes:   " << i->second.rxBytes << "\n";
        if (i->second.rxPackets > 0)
        {
            // Measure the duration of the flow from receiver's perspective
            averageFlowThroughput += i->second.rxBytes * 8.0 / flowDuration / 1000 / 1000;
            averageFlowDelay += 1000 * i->second.delaySum.GetSeconds() / i->second.rxPackets;

            outFile << "  Throughput: " << i->second.rxBytes * 8.0 / flowDuration / 1000 / 1000 << " Mbps\n";
            outFile << "  Mean delay:  " << 1000 * i->second.delaySum.GetSeconds() / i->second.rxPackets << " ms\n";
            outFile << "  Mean jitter:  " << 1000 * i->second.jitterSum.GetSeconds() / i->second.rxPackets << " ms\n";
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

    Simulator::Destroy();		// Stop the simulation

    return 0;
}

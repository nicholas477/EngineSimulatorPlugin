// Fill out your copyright notice in the Description page of Project Settings.

#include "EngineSimulator.h"
#include "EngineSimulatorPlugin.h"
#include "Sound/SoundWave.h"
#include "Sound/SoundWaveProcedural.h"
#include "HAL/FileManager.h"

#include "EngineSimulatorInternals/HeaderFixesStart.h"

#include "simulator.h"
#include "compiler.h"
#include "engine.h"
#include "transmission.h"

#include "delta.h"

#include "audio_buffer.h"

#include "EngineSimulatorInternals/HeaderFixesEnd.h"

typedef unsigned int SampleOffset;

/**
 *
 */
class FEngineSimulator : public IEngineSimulatorInterface
{
public:
    FEngineSimulator(const FEngineSimulatorParameters& InParameters);
    virtual ~FEngineSimulator();

    // IEngineSimulatorInterface
    virtual void Simulate(float DeltaTime)
    {
        process(DeltaTime);
    }

    virtual void SetDynoEnabled(bool bEnabled)
    {
        m_dynoEnabled = bEnabled;
    }

    virtual void SetStarterEnabled(bool bEnabled)
    {
        m_simulator.m_starterMotor.m_enabled = bEnabled;
    }

    virtual void SetIgnitionEnabled(bool bEnabled)
    { 
        if (m_simulator.getEngine() && m_simulator.getEngine()->getIgnitionModule())
        {
            m_simulator.getEngine()->getIgnitionModule()->m_enabled = bEnabled;
        }
    }

    virtual void SetDynoSpeed(float RPM)
    {
        m_dynoSpeed = units::rpm(RPM);
    };

    virtual void SetSpeedControl(float SpeedControl)
    { 
        if (m_iceEngine)
        {
            m_iceEngine->setSpeedControl(SpeedControl);
        }
    }

    virtual void SetGear(int32 Gear)
    {
        if (m_simulator.getTransmission())
        {
            m_simulator.getTransmission()->changeGear(Gear);
        }
    }

    virtual void SetClutchPressure(float Pressure)
    {
        if (m_simulator.getTransmission())
        {
            m_simulator.m_dyno.m_clutchPressure = FMath::Clamp(Pressure, 0.f, 1.f);
        }
    }

    virtual int32 GetGear()
    {
        if (m_simulator.getTransmission())
        {
            return m_simulator.getTransmission()->getGear();
        }

        return -1;
    }

    virtual int32 GetGearCount()
    {
        if (m_simulator.getTransmission())
        {
            return m_simulator.getTransmission()->getGearCount();
        }
        return 0;
    }

    virtual float GetSpeed()
    {
        if (m_simulator.getEngine())
        {
            return m_simulator.getEngine()->getSpeed();
        }
        return 0.f;
    }

    virtual float GetRPM()
    {
        if (m_simulator.getEngine())
        {
            return m_simulator.getEngine()->getRpm();
        }

        return 0.f;
    }

    virtual float GetRedLine()
    {
        if (m_simulator.getEngine())
        {
            return units::toRpm(m_simulator.getEngine()->getRedline());
        }
        return 0.f;
    }

    virtual float GetFilteredDynoTorque()
    {
        return m_simulator.getFilteredDynoTorque();
    }

    virtual float GetDynoPower()
    {
        return units::convert(m_simulator.getDynoPower(), units::hp);
    }

    virtual float GetGearRatio()
    {
        if (m_simulator.getTransmission())
        {
            const int32 current_gear = m_simulator.getTransmission()->getGear();
            if (current_gear >= 0 && current_gear < m_simulator.getTransmission()->getGearCount())
            {
                return m_simulator.getTransmission()->getGearRatios()[current_gear];
            }
            else
            {
                return 0.f;
            }
        }
        return 0.f;
    }

    virtual bool IsDynoEnabled()
    {
        return m_simulator.m_dyno.m_enabled;
    }

    virtual bool IsStarterEnabled()
    {
        return m_simulator.m_starterMotor.m_enabled;
    }

    virtual bool IsIgnitionEnabled()
    {
        if (m_simulator.getEngine() && m_simulator.getEngine()->getIgnitionModule())
        {
            return m_simulator.getEngine()->getIgnitionModule()->m_enabled;
        }

        return false;
    }

    virtual bool HasEngine()
    {
        return m_simulator.getEngine() != nullptr;
    }
    
    virtual FString GetName()
    {
        if (m_simulator.getEngine())
        {
            return FString(UTF8_TO_TCHAR(m_simulator.getEngine()->getName().c_str()));
        }
        return "";
    }
    // End IEngineSimulatorInterface
    
    virtual int32 GenerateAudio(float* OutAudio, int32 NumSamples) override;

protected:
    void loadScript();
    void loadEngine(Engine* engine, Vehicle* vehicle, Transmission* transmission);
    void process(float frame_dt);

protected:
    Simulator m_simulator;
    Vehicle* m_vehicle;
    Transmission* m_transmission;
    Engine* m_iceEngine;

    bool m_dynoEnabled;
    float m_dynoSpeed;
    FEngineSimulatorParameters Parameters;

    AudioBuffer m_audioBuffer;
    uint32 PlayCursor;
    std::vector<uint8> Buffer;
};

FEngineSimulator::FEngineSimulator(const FEngineSimulatorParameters& InParameters)
{
    m_vehicle = nullptr;
    m_transmission = nullptr;
    m_iceEngine = nullptr;

    m_dynoEnabled = true;
    m_dynoSpeed = 0;

    Parameters = InParameters;

    PlayCursor = 0;

	loadScript();

    m_audioBuffer.initialize(44100, 44100);
    m_audioBuffer.m_writePointer = (int)(44100 * 0.1);
}

FEngineSimulator::~FEngineSimulator()
{
    m_simulator.endAudioRenderingThread();
}

void FEngineSimulator::loadScript()
{
    UE_LOG(LogTemp, Warning, TEXT("void UEngineSimulator::loadScript()"));

    Engine* engine = nullptr;
    Vehicle* vehicle = nullptr;
    Transmission* transmission = nullptr;

#ifdef ATG_ENGINE_SIM_PIRANHA_ENABLED
    es_script::Compiler compiler;
    const FString esPath = FPaths::ConvertRelativePathToFull(FPaths::Combine(FEngineSimulatorPluginModule::GetAssetDirectory(), "../es/"));
    compiler.initialize();
    compiler.addSearchPath(TCHAR_TO_UTF8(*esPath));
    const FString EnginePath = FPaths::ConvertRelativePathToFull(FPaths::Combine(FEngineSimulatorPluginModule::GetAssetDirectory(), "main.mr"));

    if (FPaths::FileExists(TEXT("error_log.log")))
    {
        IFileManager& FileManager = IFileManager::Get();
        FileManager.Delete(TEXT("error_log.log"));
    }

    const bool compiled = compiler.compile(TCHAR_TO_UTF8(*EnginePath));
    if (compiled) {
        const es_script::Compiler::Output output = compiler.execute();

        engine = output.engine;
        vehicle = output.vehicle;
        transmission = output.transmission;
    }
    else {
        engine = nullptr;
        vehicle = nullptr;
        transmission = nullptr;
    }

    compiler.destroy();
#endif /* ATG_ENGINE_SIM_PIRANHA_ENABLED */

    if (vehicle == nullptr) {
        Vehicle::Parameters vehParams;
        vehParams.mass = units::mass(1597, units::kg);
        vehParams.diffRatio = 3.42;
        vehParams.tireRadius = units::distance(10, units::inch);
        vehParams.dragCoefficient = 0.25;
        vehParams.crossSectionArea = units::distance(6.0, units::foot) * units::distance(6.0, units::foot);
        vehParams.rollingResistance = 2000.0;
        vehicle = new Vehicle;
        vehicle->initialize(vehParams);
    }

    if (transmission == nullptr) {
        const double gearRatios[] = { 2.97, 2.07, 1.43, 1.00, 0.84, 0.56 };
        Transmission::Parameters tParams;
        tParams.GearCount = 6;
        tParams.GearRatios = gearRatios;
        tParams.MaxClutchTorque = units::torque(1000.0, units::ft_lb);
        transmission = new Transmission;
        transmission->initialize(tParams);
    }

    loadEngine(engine, vehicle, transmission);
    //refreshUserInterface();
}

void FEngineSimulator::loadEngine(Engine* engine, Vehicle* vehicle, Transmission* transmission)
{
    UE_LOG(LogTemp, Warning, TEXT("void UEngineSimulator::loadEngine()"));

    if (m_vehicle != nullptr) {
        delete m_vehicle;
        m_vehicle = nullptr;
    }

    if (m_transmission != nullptr) {
        delete m_transmission;
        m_transmission = nullptr;
    }

    if (m_iceEngine != nullptr) {
        m_iceEngine->destroy();
        delete m_iceEngine;
    }

    m_iceEngine = engine;
    m_vehicle = vehicle;
    m_transmission = transmission;

    //destroyObjects();
    m_simulator.releaseSimulation();

    if (engine == nullptr || vehicle == nullptr || transmission == nullptr) {
        m_iceEngine = nullptr;
        //m_viewParameters.Layer1 = 0;

        return;
    }

    //createObjects(engine);

    //m_viewParameters.Layer1 = engine->getMaxDepth();
    engine->calculateDisplacement();

    m_simulator.setFluidSimulationSteps(8);
    m_simulator.setSimulationFrequency(engine->getSimulationFrequency());

    Simulator::Parameters simulatorParams;
    simulatorParams.SystemType = Simulator::SystemType::NsvOptimized;
    m_simulator.initialize(simulatorParams);
    m_simulator.loadSimulation(engine, vehicle, transmission);

    Synthesizer::AudioParameters audioParams = m_simulator.getSynthesizer()->getAudioParameters();
    audioParams.InputSampleNoise = static_cast<float>(engine->getInitialJitter());
    audioParams.AirNoise = static_cast<float>(engine->getInitialNoise());
    audioParams.dF_F_mix = static_cast<float>(engine->getInitialHighFrequencyGain());
    m_simulator.getSynthesizer()->setAudioParameters(audioParams);

    m_simulator.m_dyno.m_maxTorque = m_transmission->getMaxClutchTorque();

    bool bLoadedEngineSound = false;
    for (int i = 0; i < engine->getExhaustSystemCount(); ++i) {
        ImpulseResponse* response = engine->getExhaustSystem(i)->getImpulseResponse();

        ysWindowsAudioWaveFile waveFile;
        waveFile.OpenFile(response->getFilename().c_str());
        FString AudioFilePath = UTF8_TO_TCHAR(response->getFilename().c_str());
        checkf(FPaths::FileExists(AudioFilePath), *AudioFilePath);
        UE_LOG(LogTemp, Warning, TEXT("Loading audio file: %s"), *AudioFilePath)
        waveFile.InitializeInternalBuffer(waveFile.GetSampleCount());
        waveFile.FillBuffer(0);
        waveFile.CloseFile();

        m_simulator.getSynthesizer()->initializeImpulseResponse(
            reinterpret_cast<const int16_t*>(waveFile.GetBuffer()),
            waveFile.GetSampleCount(),
            response->getVolume(),
            i
        );

        waveFile.DestroyInternalBuffer();

        bLoadedEngineSound = true;
    }

    if (bLoadedEngineSound)
    {
        m_simulator.startAudioRenderingThread();
    }
}

void FEngineSimulator::process(float frame_dt)
{
    frame_dt = static_cast<float>(clamp(frame_dt, 1 / 200.0f, 1 / 30.0f));

    if (m_transmission && m_iceEngine && m_vehicle)
    {
        m_simulator.setSimulationSpeed(1.0f);

        m_simulator.m_dyno.m_enabled = (m_simulator.getTransmission()->getGear() != -1) && m_dynoEnabled;

        //if (m_simulator.m_dyno.m_enabled) {
        //    if (!m_simulator.m_dyno.m_hold) {
        //        if (m_simulator.getFilteredDynoTorque() > units::torque(1.0, units::ft_lb)) {
        //            m_dynoSpeed += units::rpm(500) * frame_dt;
        //        }
        //        else {
        //            m_dynoSpeed *= (1 / (1 + frame_dt));
        //        }
        //    }
        //}

        //if ((m_dynoSpeed + units::rpm(1000)) > m_iceEngine->getRedline()) {
        //    m_simulator.m_dyno.m_enabled = false;
        //    m_dynoSpeed = units::rpm(0);
        //}

        //m_simulator.m_dyno.m_rotationSpeed = m_dynoSpeed + units::rpm(1000);
        m_simulator.m_dyno.m_rotationSpeed = FMath::Max(m_dynoSpeed, 0.f);
        //m_simulator.getEngine()->getIgnitionModule()->m_enabled = m_dynoSpeed > units::rpm(-100.f); // Only run ignition in forward
        m_simulator.startFrame(frame_dt);

        auto proc_t0 = std::chrono::steady_clock::now();
        const int iterationCount = m_simulator.getFrameIterationCount();
        while (m_simulator.simulateStep()) {
            //m_oscCluster->sample();
        }

        auto proc_t1 = std::chrono::steady_clock::now();

        m_simulator.endFrame();

        auto duration = proc_t1 - proc_t0;
        if (iterationCount > 0) {
            //m_performanceCluster->addTimePerTimestepSample(
            //    (duration.count() / 1E9) / iterationCount);
        }

        //const SampleOffset safeWritePosition = m_audioSource->GetCurrentWritePosition();
        //const SampleOffset writePosition = m_audioBuffer.m_writePointer;

        ////SampleOffset targetWritePosition =
        ////    m_audioBuffer.getBufferIndex(safeWritePosition, (int)(44100 * 0.1));
        //SampleOffset maxWrite = m_audioBuffer.offsetDelta(writePosition, targetWritePosition);

        //SampleOffset currentLead = m_audioBuffer.offsetDelta(safeWritePosition, writePosition);
        //SampleOffset newLead = m_audioBuffer.offsetDelta(safeWritePosition, targetWritePosition);

        //if (currentLead > 44100 * 0.5) {
        //    m_audioBuffer.m_writePointer = m_audioBuffer.getBufferIndex(safeWritePosition, (int)(44100 * 0.05));
        //    currentLead = m_audioBuffer.offsetDelta(safeWritePosition, m_audioBuffer.m_writePointer);
        //    maxWrite = m_audioBuffer.offsetDelta(m_audioBuffer.m_writePointer, targetWritePosition);
        //}

        //if (currentLead > newLead) {
        //    maxWrite = 0;
        //}

        //int16_t* samples = new int16_t[maxWrite];
        //const int readSamples = m_simulator.readAudioOutput(maxWrite, samples);

        //for (SampleOffset i = 0; i < (SampleOffset)readSamples && i < maxWrite; ++i) {
        //    const int16_t sample = samples[i];
        //    //if (m_oscillatorSampleOffset % 4 == 0) {
        //    //    m_oscCluster->getAudioWaveformOscilloscope()->addDataPoint(
        //    //        m_oscillatorSampleOffset,
        //    //        sample / (float)(INT16_MAX));
        //    //}

        //    m_audioBuffer.writeSample(sample, m_audioBuffer.m_writePointer, (int)i);

        //    //m_oscillatorSampleOffset = (m_oscillatorSampleOffset + 1) % (44100 / 10);
        //}

        //delete[] samples;

        //if (readSamples > 0) {
        //    SampleOffset size0, size1;
        //    void* data0, * data1;
        //    m_audioSource->LockBufferSegment(
        //        m_audioBuffer.m_writePointer, readSamples, &data0, &size0, &data1, &size1);

        //    m_audioBuffer.copyBuffer(
        //        reinterpret_cast<int16_t*>(data0), m_audioBuffer.m_writePointer, size0);
        //    m_audioBuffer.copyBuffer(
        //        reinterpret_cast<int16_t*>(data1),
        //        m_audioBuffer.getBufferIndex(m_audioBuffer.m_writePointer, size0),
        //        size1);

        //    m_audioSource->UnlockBufferSegments(data0, size0, data1, size1);
        //    m_audioBuffer.commitBlock(readSamples);
        //}
    }
}

const static int32 SampleRate = 44100;

int32 FEngineSimulator::GenerateAudio(float* OutAudio, int32 NumSamples)
{
    // Read out engine sound output as int16
    TArray<int16> AudioBuffer;
    AudioBuffer.AddZeroed(NumSamples);

    const uint32 SampleCount = NumSamples;
    const int ReadSamples = m_simulator.readAudioOutput(NumSamples, AudioBuffer.GetData());

    // Convert back to float
    for (int32 SampleIndex = 0; SampleIndex < NumSamples; ++SampleIndex)
    {
        OutAudio[SampleIndex] = (float)AudioBuffer[SampleIndex] / 32768.0f;
    }

    return ReadSamples;
}

TSharedPtr<IEngineSimulatorInterface> CreateEngine(const FEngineSimulatorParameters& Parameters)
{
    return MakeShared<FEngineSimulator>(Parameters);
}
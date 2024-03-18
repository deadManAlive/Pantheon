#include "Processors.h"
#include <memory>

namespace process {
    //==============================================================================
    PreProcessor::PreProcessor(AudioProcessorValueTreeState& apvts)
        : parameters(apvts)
        , preProcessorChain(new dsp::ProcessorChain<dsp::Gain<float>, dsp::Panner<float>>{})
    {
    }

    void PreProcessor::prepareToPlay(double sampleRate, int samplesPerBlock) {
        preProcessorChain->get<0>().setRampDurationSeconds((double)samplesPerBlock / sampleRate);
        preProcessorChain->get<1>().setRule(dsp::PannerRule::squareRoot3dB);

        preProcessorChain->prepare(
            {sampleRate, (uint32)samplesPerBlock, 2}
        );
    }

    void PreProcessor::processBlock(AudioSampleBuffer& buffer, MidiBuffer&) {
        updateParameter();

        dsp::AudioBlock<float>block(buffer);
        dsp::ProcessContextReplacing<float>context(block);

        preProcessorChain->process(context);
    }

    void PreProcessor::reset() {
        preProcessorChain.reset();
    }

    void PreProcessor::updateParameter() {
        const auto gainValue = parameters.getRawParameterValue("inputGain")->load();
        preProcessorChain->get<0>().setGainLinear(gainValue);

        const auto panValue = parameters.getRawParameterValue("inputPan")->load();
        preProcessorChain->get<1>().setPan(panValue);
    }

    //==============================================================================
    MixerProcessor::MixerProcessor(AudioProcessorValueTreeState& apvts)
        : parameters(apvts)
        , mixerProcessorGraph(new AudioProcessorGraph{})
    {
    }

    void MixerProcessor::prepareToPlay(double sampleRate, int samplesPerBlock) {
        //==============================================================================
        mixerProcessorGraph->setPlayConfigDetails(getMainBusNumInputChannels(),
                                        getMainBusNumOutputChannels(),
                                        sampleRate, samplesPerBlock);
        mixerProcessorGraph->prepareToPlay(sampleRate, samplesPerBlock);
        mixerProcessorGraph->clear();

        //==============================================================================
        audioInputNode = mixerProcessorGraph->addNode(std::make_unique<IOProcessor>(IOProcessor::audioInputNode));
        audioOutputNode = mixerProcessorGraph->addNode(std::make_unique<IOProcessor>(IOProcessor::audioOutputNode));

        leftPreGainUnitNode = mixerProcessorGraph->addNode(std::make_unique<MixerUnit<Left, Left>>(parameters));
        leftToRightGainUnitNode = mixerProcessorGraph->addNode(std::make_unique<MixerUnit<Left, Right>>(parameters));
        rightToLeftGainUnitNode = mixerProcessorGraph->addNode(std::make_unique<MixerUnit<Right, Left>>(parameters));
        rightPreGainUnitNode = mixerProcessorGraph->addNode(std::make_unique<MixerUnit<Right, Right>>(parameters));

        //==============================================================================
        for (int ch = 0; ch < 2; ++ch) {
            if (ch == Left) {
                // IN.L -> LL
                mixerProcessorGraph->addConnection({{audioInputNode->nodeID, ch}, 
                                            {leftPreGainUnitNode->nodeID, 0}});

                // IN.L -> LR
                mixerProcessorGraph->addConnection({{audioInputNode->nodeID, ch},
                                            {leftToRightGainUnitNode->nodeID, 0}});
                
                // LL => OUT.L
                mixerProcessorGraph->addConnection({{leftPreGainUnitNode->nodeID, 0},                
                                            {audioOutputNode->nodeID, ch}});
                
                // RL => OUT.L
                mixerProcessorGraph->addConnection({{rightToLeftGainUnitNode->nodeID, 0},                
                                            {audioOutputNode->nodeID, ch}});
            } else {
                // IN.R -> RR
                mixerProcessorGraph->addConnection({{audioInputNode->nodeID, ch},
                                            {rightPreGainUnitNode->nodeID, 0}});

                // IN.R -> RL
                mixerProcessorGraph->addConnection({{audioInputNode->nodeID, ch},
                                            {rightToLeftGainUnitNode->nodeID, 0}});
                
                // RR => OUT.R
                mixerProcessorGraph->addConnection({{rightPreGainUnitNode->nodeID, 0},                
                                            {audioOutputNode->nodeID, ch}});

                // LR => OUT.R
                mixerProcessorGraph->addConnection({{leftToRightGainUnitNode->nodeID, 0},                
                                            {audioOutputNode->nodeID, ch}});
            }
        }
    }

    void MixerProcessor::processBlock(AudioSampleBuffer& buffer, MidiBuffer& midiMessages) {
        mixerProcessorGraph->processBlock(buffer, midiMessages);
    }

    void MixerProcessor::reset() {
        mixerProcessorGraph->reset();
    }

    //==============================================================================
    FxProcessor::FxProcessor(AudioProcessorValueTreeState& apvts)
        : parameters(apvts)
        , fxProcessorGraph(new AudioProcessorGraph{})
    {
    }

    void FxProcessor::prepareToPlay(double sampleRate, int samplesPerBlock) {
        //==============================================================================
        fxProcessorGraph->setPlayConfigDetails(getMainBusNumInputChannels(),
                                        getMainBusNumOutputChannels(),
                                        sampleRate, samplesPerBlock);
        fxProcessorGraph->prepareToPlay(sampleRate, samplesPerBlock);
        fxProcessorGraph->clear();
    
        //==============================================================================
        audioInputNode = fxProcessorGraph->addNode(std::make_unique<IOProcessor>(IOProcessor::audioInputNode));
        leftFxNode = fxProcessorGraph->addNode(std::make_unique<LeftFxUnit>(parameters));
        rightFxNode = fxProcessorGraph->addNode(std::make_unique<RightFxUnit>(parameters));
        audioOutputNode = fxProcessorGraph->addNode(std::make_unique<IOProcessor>(IOProcessor::audioOutputNode));
    
        // IN.L -> FXL
        fxProcessorGraph->addConnection({
            {audioInputNode->nodeID, 0},
            {leftFxNode->nodeID, 0},
        });

        // IN.R -> FXR
        fxProcessorGraph->addConnection({
            {audioInputNode->nodeID, 1},
            {rightFxNode->nodeID, 0},
        });

        // FXL -> OUT.L
        fxProcessorGraph->addConnection({
            {leftFxNode->nodeID, 0},
            {audioOutputNode->nodeID, 0},
        });

        // FXR -> OUT.R
        fxProcessorGraph->addConnection({
            {rightFxNode->nodeID, 0},
            {audioOutputNode->nodeID, 1},
        });
    }

    void FxProcessor::processBlock(AudioSampleBuffer& buffer, MidiBuffer& midiMessages) {
        ignoreUnused(buffer);
        
        fxProcessorGraph->processBlock(buffer, midiMessages);
    }

    void FxProcessor::reset() {
        fxProcessorGraph->reset();
    }


    // FxProcessor::FxUnit::FxUnit(AudioProcessorValueTreeState& apvts)
    //     : parameters(apvts)
    //     , fxUnitProcessor(new FxProcess{})
    // {
    // }

    // void FxProcessor::FxUnit::prepareToPlay(double sampleRate, int samplesPerBlock) {
    //     fxUnitProcessor->get<0>().setMaximumDelayInSamples(samplesPerBlock / 2);
    //     fxUnitProcessor->prepare(
    //         {sampleRate, (uint32)samplesPerBlock, 1}
    //     );
    // }

    // void FxProcessor::FxUnit::processBlock(AudioSampleBuffer& buffer, MidiBuffer&) {
    //     updateParameter();

    //     dsp::AudioBlock<float>block(buffer);
    //     dsp::ProcessContextReplacing<float>context(block);

    //     fxUnitProcessor->process(context);
    // }

    // void FxProcessor::FxUnit::reset() {
    //     fxUnitProcessor->reset();
    // }

    // void FxProcessor::FxUnit::updateParameter() {

    // }
}
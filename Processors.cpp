#include "Processors.h"

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
        , mixerProcessor(new AudioProcessorGraph{})
    {
    }

    void MixerProcessor::prepareToPlay(double sampleRate, int samplesPerBlock) {
        //==============================================================================
        mixerProcessor->setPlayConfigDetails(getMainBusNumInputChannels(),
                                        getMainBusNumOutputChannels(),
                                        sampleRate, samplesPerBlock);
        mixerProcessor->prepareToPlay(sampleRate, samplesPerBlock);
        mixerProcessor->clear();

        //==============================================================================
        audioInputNode = mixerProcessor->addNode(std::make_unique<IOProcessor>(IOProcessor::audioInputNode));
        audioOutputNode = mixerProcessor->addNode(std::make_unique<IOProcessor>(IOProcessor::audioOutputNode));

        leftPreGainUnitNode = mixerProcessor->addNode(std::make_unique<MixerUnit<Left, Left>>(parameters));
        leftToRightGainUnitNode = mixerProcessor->addNode(std::make_unique<MixerUnit<Left, Right>>(parameters));
        rightToLeftGainUnitNode = mixerProcessor->addNode(std::make_unique<MixerUnit<Right, Left>>(parameters));
        rightPreGainUnitNode = mixerProcessor->addNode(std::make_unique<MixerUnit<Right, Right>>(parameters));

        //==============================================================================
        for (int ch = 0; ch < 2; ++ch) {
            if (ch == Left) {
                // IN.L -> LL
                mixerProcessor->addConnection({{audioInputNode->nodeID, ch}, 
                                            {leftPreGainUnitNode->nodeID, 0}});

                // IN.L -> LR
                mixerProcessor->addConnection({{audioInputNode->nodeID, ch},
                                            {leftToRightGainUnitNode->nodeID, 0}});
                
                // LL => OUT.L
                mixerProcessor->addConnection({{leftPreGainUnitNode->nodeID, 0},                
                                            {audioOutputNode->nodeID, ch}});
                
                // RL => OUT.L
                mixerProcessor->addConnection({{rightToLeftGainUnitNode->nodeID, 0},                
                                            {audioOutputNode->nodeID, ch}});
            } else {
                // IN.R -> RR
                mixerProcessor->addConnection({{audioInputNode->nodeID, ch},
                                            {rightPreGainUnitNode->nodeID, 0}});

                // IN.R -> RL
                mixerProcessor->addConnection({{audioInputNode->nodeID, ch},
                                            {rightToLeftGainUnitNode->nodeID, 0}});
                
                // RR => OUT.R
                mixerProcessor->addConnection({{rightPreGainUnitNode->nodeID, 0},                
                                            {audioOutputNode->nodeID, ch}});

                // LR => OUT.R
                mixerProcessor->addConnection({{leftToRightGainUnitNode->nodeID, 0},                
                                            {audioOutputNode->nodeID, ch}});
            }
        }
    }

    void MixerProcessor::processBlock(AudioSampleBuffer& buffer, MidiBuffer& midiMessages) {
        mixerProcessor->processBlock(buffer, midiMessages);
    }

    void MixerProcessor::reset() {
        mixerProcessor->reset();
    }
}
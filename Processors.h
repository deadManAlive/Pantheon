#pragma once

#include "juce_core/system/juce_PlatformDefs.h"
#include <JuceHeader.h>
#include <memory>

//==============================================================================
class PantheonProcessorBase  : public juce::AudioProcessor
{
public:
    //==============================================================================
    PantheonProcessorBase(BusesProperties ioLayouts
        = BusesProperties().withInput ("Input", juce::AudioChannelSet::stereo())
                                           .withOutput ("Output", juce::AudioChannelSet::stereo()))
        : AudioProcessor (ioLayouts)
    {}
    //==============================================================================
    void prepareToPlay (double, int) override {}
    void releaseResources() override {}
    void processBlock (juce::AudioSampleBuffer&, juce::MidiBuffer&) override {}

    //==============================================================================
    juce::AudioProcessorEditor* createEditor() override          { return nullptr; }
    bool hasEditor() const override                              { return false; }

    //==============================================================================
    const juce::String getName() const override                  { return {}; }
    bool acceptsMidi() const override                            { return false; }
    bool producesMidi() const override                           { return false; }
    double getTailLengthSeconds() const override                 { return 0; }

    //==============================================================================
    int getNumPrograms() override                                { return 0; }
    int getCurrentProgram() override                             { return 0; }
    void setCurrentProgram (int) override                        {}
    const juce::String getProgramName (int) override             { return {}; }
    void changeProgramName (int, const juce::String&) override   {}

    //==============================================================================
    void getStateInformation (juce::MemoryBlock&) override       {}
    void setStateInformation (const void*, int) override         {}

private:
    //==============================================================================
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (PantheonProcessorBase)
};

namespace process {
    //==============================================================================
    class PreProcessor : public PantheonProcessorBase {
    public:
        PreProcessor(AudioProcessorValueTreeState&);
        void prepareToPlay(double, int) override;
        void processBlock(AudioSampleBuffer&, MidiBuffer&) override;
        void reset() override;
        const String getName() const override {return "Pre";}
    private:
        //==============================================================================
        AudioProcessorValueTreeState& parameters;
        dsp::ProcessorChain<dsp::Gain<float>, dsp::Panner<float>> preProcessorChain;
        void updateParameter();

        //==============================================================================
        JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (PreProcessor)
    };

    //==============================================================================
    enum Channel {
        Left = 0,
        Right = 1,
    };

    template <Channel SOURCE, Channel TARGET>
    class MixerUnit : public PantheonProcessorBase {
    public:
        MixerUnit(AudioProcessorValueTreeState& apvts)
            : PantheonProcessorBase(BusesProperties().withInput ("Input", juce::AudioChannelSet::mono())
                                           .withOutput ("Output", juce::AudioChannelSet::mono()))
            , parameters(apvts)
        {
        }

        void prepareToPlay(double sampleRate, int samplesPerBlock) override {
            gain.setRampDurationSeconds((double)samplesPerBlock / sampleRate);
            gain.prepare(
                {sampleRate, (uint32)samplesPerBlock, 1}
            );
        }

        void processBlock(AudioSampleBuffer& buffer, MidiBuffer&) override {
            updateParameter();

            dsp::AudioBlock<float>block(buffer);
            dsp::ProcessContextReplacing<float>context(block);

            gain.process(context);
        }

        void reset() override {
            gain.reset();
        }

        const String getName() const override {return "MixerUnit";}

    private:
        //==============================================================================
        AudioProcessorValueTreeState& parameters;
        dsp::Gain<float> gain;

        //==============================================================================
        static constexpr const char* directions[4] = {"leftPreGain",
                                                     "leftToRightGain",
                                                     "rightToLeftGain",
                                                     "rightPreGain"};

        static constexpr const char* z = directions[(SOURCE << 1) | TARGET];

        void updateParameter() {
            const auto gainValue = parameters.getRawParameterValue(z)->load();
            gain.setGainLinear(gainValue);
        }

        JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (MixerUnit)
    };

    // //==============================================================================
    // template<Channel C>
    // class PostProcessor : public PantheonProcessorBase {
    // public:
    //     PostProcessor(AudioProcessorValueTreeState& apvts)
    //         : parameters(apvts)
    //     {
    //     }

    //     void prepareToPlay(double sampleRate, int samplesPerBlock) override {
    //         panner.setRule(juce::dsp::PannerRule::balanced);
    //         panner.prepare(
    //         {sampleRate, (uint32)samplesPerBlock, 2}
    //         );
    //     }
        
    //     void processBlock(AudioSampleBuffer& buffer, MidiBuffer&) override {
    //         updateParameter();

    //         dsp::AudioBlock<float>block(buffer);
    //         dsp::ProcessContextReplacing<float>context(block);

    //         panner.process(context);
    //     }

    //     void reset() override {
    //         panner.reset();
    //     }
        
    //     const String getName() const override {
    //         return "Post";
    //     }
    // private:
    //     //==============================================================================
    //     AudioProcessorValueTreeState& parameters;
    //     dsp::Panner<float> panner;
        
    //     //==============================================================================
    //     static constexpr const char* z = C == Channel::Left ? "leftPan" : "rightPan";

    //     //==============================================================================
    //     void updateParameter() {
    //         const auto panValue = parameters.getRawParameterValue(z)->load();
    //         panner.setPan(panValue);
    //     }

    //     //==============================================================================
    //     JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (PostProcessor)
    // };

    //==============================================================================
    class MixerProcessor : public PantheonProcessorBase {
    public:
        MixerProcessor(AudioProcessorValueTreeState&);
        void prepareToPlay(double, int) override;
        void processBlock(AudioSampleBuffer&, MidiBuffer&) override;
        void reset() override;
        const String getName() const override {return "Mixer";}
    private:
        AudioProcessorValueTreeState& parameters;

        //==============================================================================
        std::unique_ptr<AudioProcessorGraph> mixerProcessor;

        using IOProcessor = AudioProcessorGraph::AudioGraphIOProcessor;
        using Node = AudioProcessorGraph::Node;

        Node::Ptr audioInputNode;

        Node::Ptr leftPreGainUnitNode;
        Node::Ptr leftToRightGainUnitNode;
        Node::Ptr rightToLeftGainUnitNode;
        Node::Ptr rightPreGainUnitNode;

        Node::Ptr audioOutputNode;

        //==============================================================================
        JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (MixerProcessor)
    };
}

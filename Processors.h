#pragma once

#include <JuceHeader.h>
#include <atomic>
#include <atomic>
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
        std::unique_ptr<dsp::ProcessorChain<dsp::Gain<float>, dsp::Panner<float>>> preProcessorChain;
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
            , gain(new dsp::Gain<float>{})
        {
        }

        void prepareToPlay(double sampleRate, int samplesPerBlock) override {
            gain->setRampDurationSeconds((double)samplesPerBlock / sampleRate);
            gain->prepare(
                {sampleRate, (uint32)samplesPerBlock, 1}
            );
        }

        void processBlock(AudioSampleBuffer& buffer, MidiBuffer&) override {
            updateParameter();

            dsp::AudioBlock<float>block(buffer);
            dsp::ProcessContextReplacing<float>context(block);

            gain->process(context);
        }

        void reset() override {
            gain.reset();
        }

        const String getName() const override {return "MixerUnit";}

    private:
        //==============================================================================
        AudioProcessorValueTreeState& parameters;
        std::unique_ptr<dsp::Gain<float>> gain;

        //==============================================================================
        static constexpr const char* directions[4] = {"leftPreGain",
                                                     "leftToRightGain",
                                                     "rightToLeftGain",
                                                     "rightPreGain"};

        static constexpr const char* z = directions[(SOURCE << 1) | TARGET];

        void updateParameter() {
            const auto gainValue = parameters.getRawParameterValue(z)->load();
            gain->setGainLinear(gainValue);
        }

        JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (MixerUnit)
    };

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
        std::unique_ptr<AudioProcessorGraph> mixerProcessorGraph;

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

    //==============================================================================
    class FxProcessor : public PantheonProcessorBase {
    public:
        FxProcessor(AudioProcessorValueTreeState&);
        void prepareToPlay(double, int) override;
        void processBlock(AudioSampleBuffer&, MidiBuffer&) override;
        void reset() override;
        const String getName() const override {return "Fx";}
    private:
        AudioProcessorValueTreeState& parameters;


        //==============================================================================
        template <Channel CHANNEL>
        class FxUnit : public PantheonProcessorBase {
        public:
            FxUnit(AudioProcessorValueTreeState& apvts)
                : PantheonProcessorBase(BusesProperties().withInput ("Input", juce::AudioChannelSet::mono())
                                           .withOutput ("Output", juce::AudioChannelSet::mono()))
                , parameters(apvts)
                , fxUnitProcessor(new FxProcess{})
            { 
            }

            void prepareToPlay(double sampleRate, int samplesPerBlock) override {
                maxDelayInSamples = samplesPerBlock / 2;
                _sampleRate = sampleRate / 2;

                delayParamSmoothedValue.reset(samplesPerBlock / 4);
                filterParamSmoothedValue.reset(samplesPerBlock / 4);

                fxUnitProcessor->get<0>().setMaximumDelayInSamples(maxDelayInSamples / 2);
                fxUnitProcessor->get<1>().coefficients = Coefficients::makeAllPass(sampleRate, 10);
                fxUnitProcessor->prepare(
                    {sampleRate, (uint32)samplesPerBlock, 1}
                );
                fxUnitProcessor->reset();
            }

            void processBlock(AudioSampleBuffer& buffer, MidiBuffer&) override {
                updateParameter();

                dsp::AudioBlock<float>block(buffer);
                dsp::ProcessContextReplacing<float>context(block);

                fxUnitProcessor->process(context);
            }

            void reset() override {
                fxUnitProcessor->reset();
            }
        private:
            AudioProcessorValueTreeState& parameters;

            //==============================================================================
            int maxDelayInSamples { 128 };
            double _sampleRate { 44100. };

            //==============================================================================
            using FxProcess = dsp::ProcessorChain<dsp::DelayLine<float>, dsp::IIR::Filter<float>>;
            using Coefficients = dsp::IIR::Coefficients<float>;

            std::unique_ptr<FxProcess> fxUnitProcessor;

            //==============================================================================
            LinearSmoothedValue<float> delayParamSmoothedValue;
            LinearSmoothedValue<float> filterParamSmoothedValue;

            //==============================================================================
            // TODO: logarithmize all pass filter slider + why the shit is it inverted in FL?
            void updateParameter() {
                const auto delayParam = parameters.getRawParameterValue("delayLine");
                const auto filterParam = parameters.getRawParameterValue("allPassFreq");

                const auto delayParamValue = delayParam->load();
                const auto filterParamValue = filterParam->load();

                delayParamSmoothedValue.setTargetValue(delayParamValue);
                filterParamSmoothedValue.setTargetValue(filterParamValue);

                const auto currentDelayValue = delayParamSmoothedValue.getNextValue();
                const auto currentFilterValue = filterParamSmoothedValue.getNextValue();

                float delay;
                float filter;

                if (CHANNEL == Left) {
                    delay = abs(jlimit(-1.f, 0.f, currentDelayValue)) * static_cast<float>(maxDelayInSamples);
                    filter = abs(jlimit(-1.f, 0.f, currentFilterValue)) * static_cast<float>(_sampleRate / 2);
                } else {
                    delay = jlimit(0.f, 1.f, currentDelayValue) * static_cast<float>(maxDelayInSamples);
                    filter = jlimit(0.f, 1.f, currentDelayValue) * static_cast<float>(_sampleRate / 2);
                }

                fxUnitProcessor->get<0>().setDelay(delay);
                fxUnitProcessor->get<1>().coefficients = Coefficients::makeAllPass(_sampleRate, filter > 10.f ? filter : 10.f);
            }

            //==============================================================================
            JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (FxUnit)
        };

        using LeftFxUnit = FxUnit<Left>;
        using RightFxUnit = FxUnit<Right>;
        
        //==============================================================================
        std::unique_ptr<AudioProcessorGraph> fxProcessorGraph;

        using IOProcessor = AudioProcessorGraph::AudioGraphIOProcessor;
        using Node = AudioProcessorGraph::Node;

        Node::Ptr audioInputNode;
        Node::Ptr leftFxNode;
        Node::Ptr rightFxNode;
        Node::Ptr audioOutputNode;

        //==============================================================================
        JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (FxProcessor)
    };
}

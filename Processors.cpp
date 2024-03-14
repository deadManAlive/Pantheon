#include "Processors.h"

namespace process {
    PreProcessor::PreProcessor(AudioProcessorValueTreeState& apvts)
        : parameters(apvts)
    {
    }

    void PreProcessor::prepareToPlay(double sampleRate, int samplesPerBlock) {
        preProcessorChain.get<0>().setRampDurationSeconds((double)samplesPerBlock / sampleRate);
        preProcessorChain.get<1>().setRule(dsp::PannerRule::squareRoot3dB);

        preProcessorChain.prepare(
            {sampleRate, (uint32)samplesPerBlock, 2}
        );
    }

    void PreProcessor::processBlock(AudioSampleBuffer& buffer, MidiBuffer&) {
        updateParameter();

        dsp::AudioBlock<float>block(buffer);
        dsp::ProcessContextReplacing<float>context(block);

        preProcessorChain.process(context);
    }

    void PreProcessor::reset() {
        preProcessorChain.reset();
    }

    void PreProcessor::updateParameter() {
        const auto gainValue = parameters.getRawParameterValue("inputGain")->load();
        preProcessorChain.get<0>().setGainLinear(gainValue);

        const auto panValue = parameters.getRawParameterValue("inputPan")->load();
        preProcessorChain.get<1>().setPan(panValue);
    }
}
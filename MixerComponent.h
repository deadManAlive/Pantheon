#pragma once

#include <JuceHeader.h>

#include "LookAndFeel.h"
#include "PluginProcessor.h"

class MixerComponent : public juce::Component
{
public:
    MixerComponent(AudioPluginAudioProcessor&, AudioProcessorValueTreeState&);
    ~MixerComponent() override;
    void paint(juce::Graphics& g) override;
    void resized() override;
private:
    AudioPluginAudioProcessor& processorRef;
    AudioProcessorValueTreeState& parameters;

    //==============================================================================
    using SliderAttachment = AudioProcessorValueTreeState::SliderAttachment;
    using SliderAttachmentPtr = std::unique_ptr<SliderAttachment>;

    PanLook leftPanLook;
    PanLook rightPanLook;

    Slider rightToLeftGainSlider;
    Slider leftPreGainSlider;
    Slider rightPreGainSlider;
    Slider leftToRightGainSlider;

    SliderAttachmentPtr rightToLeftGainAttachment = nullptr;
    SliderAttachmentPtr leftPreGainAttachment = nullptr;
    SliderAttachmentPtr rightPreGainAttachment = nullptr;
    SliderAttachmentPtr leftToRightGainAttachment = nullptr;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (MixerComponent)
};

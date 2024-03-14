#pragma once

#include <JuceHeader.h>

#include "LookAndFeel.h"
#include "PluginProcessor.h"

class PreComponent : public juce::Component
{
public:
    PreComponent(AudioPluginAudioProcessor&, AudioProcessorValueTreeState&);
    ~PreComponent() override;
    void paint(juce::Graphics& g) override;
    void resized() override;
private:
    AudioPluginAudioProcessor& processorRef;
    AudioProcessorValueTreeState& parameters;

    //==============================================================================
    using SliderAttachment = AudioProcessorValueTreeState::SliderAttachment;
    using SliderAttachmentPtr = std::unique_ptr<SliderAttachment>;

    PanLook panLook;
    PanLook volLook;

    Slider inputGainSlider;
    Slider inputPanSlider;

    SliderAttachmentPtr inputGainAttachment;
    SliderAttachmentPtr inputPanAttachment;

    GroupComponent border;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (PreComponent)
};

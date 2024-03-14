#pragma once

#include <JuceHeader.h>

#include "LookAndFeel.h"
#include "PluginProcessor.h"

class PostComponent : public juce::Component
{
public:
    PostComponent(AudioPluginAudioProcessor&, AudioProcessorValueTreeState&);
    ~PostComponent() override;
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
    
    Slider leftPostPanSlider;
    Slider rightPostPanSlider;

    SliderAttachmentPtr leftPostPanAttachment;
    SliderAttachmentPtr rightPostPanAttachment;

    GroupComponent border;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (PostComponent)
};

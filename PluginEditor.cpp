#include "LookAndFeel.h"
#include "PluginProcessor.h"
#include "PluginEditor.h"

//==============================================================================
AudioPluginAudioProcessorEditor::AudioPluginAudioProcessorEditor (AudioPluginAudioProcessor& p, AudioProcessorValueTreeState& apvts)
    : AudioProcessorEditor (&p)
    , mixerComponent(p, apvts)
    , preComponent(p, apvts)
    , fxComponent(p, apvts)
{
    panLook.setColour(GroupComponent::outlineColourId, Colours::linen);
    panLook.setColour(GroupComponent::textColourId, Colours::linen);

    panLook.textH = 20.f;
    border.setLookAndFeel(&panLook);
    border.setText("Pantheon");

    addAndMakeVisible(preComponent);
    addAndMakeVisible(mixerComponent);
    // addAndMakeVisible(postComponent);
    addAndMakeVisible(filler);
    addAndMakeVisible(fxComponent);

    addAndMakeVisible(border);

    double ratio = 1./2.;
    int min_height = 200;
    int max_height = 1080;
    int default_size = 300;
    setResizeLimits(min_height, (int)(min_height/ratio), max_height, (int)(max_height/ratio));
    getConstrainer()->setFixedAspectRatio(ratio);
    setSize(default_size, (int)(default_size/ratio));
}

AudioPluginAudioProcessorEditor::~AudioPluginAudioProcessorEditor()
{
}

//==============================================================================
void AudioPluginAudioProcessorEditor::paint (juce::Graphics& g)
{
    // (Our component is opaque, so we must completely fill the background with a solid colour)
    g.fillAll (getLookAndFeel().findColour (juce::ResizableWindow::backgroundColourId));
}

void FillerComp::paint(Graphics& g) {
    const auto bounds = getLocalBounds().toFloat();
    const auto w = bounds.getWidth();
    const auto h = bounds.getHeight();
    const auto headDim = jmin(8.f, 0.2f * h);

    // RIGHT ARROW
    Point<float> r0{0.5625f * w, 0.f};
    Point<float> r1{r0.x, 0.75f * h};
    Point<float> r2{0.3125f * w, r1.y};
    Point<float> r3{r2.x, 0.f};

    Path rightArrow;

    rightArrow.startNewSubPath(r0);
    rightArrow.lineTo(r1);
    rightArrow.lineTo(r2);
    rightArrow.addArrow({r2, r3}, 0.f, headDim, headDim);

    g.setColour(PanLook::rightColour);
    g.strokePath(rightArrow, {2.f, PathStrokeType::mitered, PathStrokeType::butt});

    // LEFT ARROW
    Point<float> l0{0.4375f * w, 0.f};
    Point<float> l1{l0.x, 0.50f * h};
    Point<float> l2{0.6875f * w, l1.y};
    Point<float> l3{l2.x, 0.f};

    Path leftArrow;

    leftArrow.startNewSubPath(l0);
    leftArrow.lineTo(l1);
    leftArrow.lineTo(l2);
    leftArrow.addArrow({l2, l3}, 0.f, headDim, headDim);

    g.setColour(PanLook::leftColour);
    g.strokePath(leftArrow, {2.f, PathStrokeType::mitered, PathStrokeType::butt});
}

void AudioPluginAudioProcessorEditor::resized()
{
    // This is generally where you'll want to lay out the positions of any
    // subcomponents in your editor..
    Grid grid;

    using Track = Grid::TrackInfo;
    using Fr = Grid::Fr;

    grid.templateRows = {
        Track(Fr(5)),
        Track(Fr(10)),
        Track(Fr(1)),
        // Track(Fr(3)),
    };
    grid.templateColumns = {
        Track(Fr(3)),
        Track(Fr(5))
    };

    grid.items = {
        GridItem(preComponent),
        GridItem(fxComponent),
        GridItem(mixerComponent).withArea(2, GridItem::Span(2)),
        GridItem(filler).withArea(3, GridItem::Span(2)),
    };

    border.setBounds(getLocalBounds().reduced(4));
    grid.performLayout(getLocalBounds().reduced(16));
}
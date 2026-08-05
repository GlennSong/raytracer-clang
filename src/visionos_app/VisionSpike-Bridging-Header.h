// Swift -> C++ bridge for the visionOS spike.
//
// Exposes rt_vision_spike_run to VisionSpikeApp.swift. The cp_layer_renderer_t
// parameter needs no conversion: it is an Objective-C object whose Swift name
// is `LayerRenderer`, so the value the CompositorLayer closure yields is
// already the right type.

#import "vision_spike.h"

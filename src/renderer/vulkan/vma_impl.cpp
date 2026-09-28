// The one translation unit that compiles the Vulkan Memory Allocator's implementation
// (third_party/VulkanMemoryAllocator, v3.3.0, MIT). vulkan_renderer.cpp includes the
// header with the same configuration. ADR-0094.
#define VMA_IMPLEMENTATION
#define VMA_STATIC_VULKAN_FUNCTIONS 1
#define VMA_DYNAMIC_VULKAN_FUNCTIONS 0
#include "vk_mem_alloc.h"

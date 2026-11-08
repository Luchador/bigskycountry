#include "sun_shader.hpp"

#ifdef BIGSKY_GPU_SUN
namespace
{
    alignas(4)
    #include "sun_dxil.hpp"
    alignas(4)
    #include "sun_spirv.hpp"
}
#endif

SunShader::~SunShader()
{
    SDL_DestroyGPURenderState(state);
    if (shader)
    {
        SDL_ReleaseGPUShader(device, shader);
    }
}

bool SunShader::Initialize(SDL_Renderer* renderer, SDL_Texture* mask)
{
#ifdef BIGSKY_GPU_SUN
    device = static_cast<SDL_GPUDevice*>(SDL_GetPointerProperty(SDL_GetRendererProperties(renderer),
                                                                SDL_PROP_RENDERER_GPU_DEVICE_POINTER, nullptr));
    if (device)
    {
        const SDL_GPUShaderFormat formats = SDL_GetGPUShaderFormats(device);
        SDL_GPUShaderCreateInfo info{};
        if (formats & SDL_GPU_SHADERFORMAT_DXIL)
        {
            info.format = SDL_GPU_SHADERFORMAT_DXIL;
            info.code = sunDxil;
            info.code_size = sizeof(sunDxil);
        }
        else if (formats & SDL_GPU_SHADERFORMAT_SPIRV)
        {
            info.format = SDL_GPU_SHADERFORMAT_SPIRV;
            info.code = sunSpirv;
            info.code_size = sizeof(sunSpirv);
        }
        if (info.code)
        {
            if (!SDL_SetTextureBlendMode(mask, SDL_BLENDMODE_ADD_PREMULTIPLIED))
            {
                return false;
            }
            info.entrypoint = "main";
            info.stage = SDL_GPU_SHADERSTAGE_FRAGMENT;
            info.num_samplers = 1;
            shader = SDL_CreateGPUShader(device, &info);
            if (!shader)
            {
                return false;
            }
            SDL_GPURenderStateCreateInfo stateInfo{};
            stateInfo.fragment_shader = shader;
            state = SDL_CreateGPURenderState(renderer, &stateInfo);
            if (!state)
            {
                SDL_ReleaseGPUShader(device, shader);
                shader = nullptr;
                return false;
            }
            SDL_Log("Sun uses the additive HLSL shader on %s", SDL_GetGPUDeviceDriver(device));
            return true;
        }
    }
#endif
    // The built-in renderer returns straight RGB and mask alpha, so its blend mode
    // applies coverage once. The custom shader applies coverage before blending.
    if (!SDL_SetTextureBlendMode(mask, SDL_BLENDMODE_ADD))
    {
        return false;
    }
    SDL_Log("Sun uses built-in additive blending on %s", SDL_GetRendererName(renderer));
    return true;
}

bool SunShader::Draw(SDL_Renderer* renderer, SDL_Texture* mask, const SDL_Vertex* vertices, int vertexCount,
                     const int* indices, int indexCount) const
{
    if (state && !SDL_SetGPURenderState(renderer, state))
    {
        return false;
    }
    const bool drawn = SDL_RenderGeometry(renderer, mask, vertices, vertexCount, indices, indexCount);
    if (state && !SDL_SetGPURenderState(renderer, nullptr))
    {
        return false;
    }
    return drawn;
}

#pragma once

#include "./mLibInclude.h"
#include "assetManager.h"

struct AssetRendererConstantBuffer {
  ml::mat4f worldViewProj;
  ml::vec4f modelColor;
};

class AssetRenderer {
 public:

  void init(ml::GraphicsDevice& g);

  void renderModel(AssetManager& assets, ml::GraphicsDevice& g, const ml::mat4f& cameraPerspective,
                   const std::string& modelId, const ml::mat4f& modelToWorld,
                   const ml::vec4f& color = ml::vec4f(1, 1, 1, 1));

  void renderMesh(ml::GraphicsDevice& g, const ml::mat4f& cameraPerspective, const ml::mat4f& meshToWorld,
                  const ml::D3D11TriMesh& mesh, const ml::vec4f& color = ml::vec4f(1, 1, 1, 1));

  void renderCylinder(ml::GraphicsDevice& g, const ml::mat4f& cameraPerspective, const ml::vec3f& p0,
                      const ml::vec3f& p1, const ml::vec4f& color);

  void renderSphere(ml::GraphicsDevice& g, const ml::mat4f& cameraPerspective, const ml::vec3f& center, float radius,
                    const ml::vec4f& color);

  void renderBox(ml::GraphicsDevice& g, const ml::mat4f& cameraPerspective, const ml::vec3f& center, float radius,
                 const ml::vec4f& color);

  void renderPolygon(ml::GraphicsDevice& g, const ml::mat4f& cameraPerspective, const ml::Polygonf& poly,
                     const ml::vec4f& color, float height);

 private:
  ml::D3D11VertexShader m_vsColor, m_vsTexture;
  ml::D3D11PixelShader m_psColor, m_psTexture;
  ml::D3D11ConstantBuffer<AssetRendererConstantBuffer> m_constants;

  ml::D3D11TriMesh m_cylinder;
  ml::D3D11TriMesh m_sphere;
  ml::D3D11TriMesh m_box;
};

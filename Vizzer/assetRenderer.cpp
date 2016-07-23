#include "common.h"  // NOLINT
#include "assetRenderer.h"

using ml::mat4f;
using ml::vec3f;
using ml::vec4f;
using std::string;

void AssetRenderer::init(ml::GraphicsDevice& _g) {
  ml::GraphicsDevice& g = _g.castD3D11();
  m_vsColor.load(g, "../../shaders/modelRendererColor.shader");
  m_psColor.load(g, "../../shaders/modelRendererColor.shader");
  m_vsTexture.load(g, "../../shaders/modelRendererTexture.shader");
  m_psTexture.load(g, "../../shaders/modelRendererTexture.shader");
  m_constants.init(g);

  m_sphere.load(g, ml::Shapesf::sphere(1.0f, ml::vec3f::origin));
  //m_cylinder.load(g, ml::Shapesf::cylinder(0.02f, 0.35f, 2, 15, ml::vec4f(1.0f, 1.0f, 1.0f, 1.0f)));
  m_cylinder.load(g, ml::Shapesf::cylinder(0.01f, 1.0f, 2, 15, ml::vec4f(1.0f, 1.0f, 1.0f, 1.0f)));
  m_box.load(g, ml::Shapesf::box(1.0f));
}

void AssetRenderer::renderModel(AssetManager& assets, ml::GraphicsDevice& g, const mat4f& cameraPerspective,
                                const string& modelId, const mat4f& modelToWorld, const vec4f& color) {
  m_vsColor.bind();
  m_psColor.bind();

  AssetRendererConstantBuffer constants;

  constants.worldViewProj = cameraPerspective * modelToWorld;
  constants.modelColor = color;
  m_constants.updateAndBind(constants, 0);
  assets.loadModel(g, modelId).mesh.render();
}

void AssetRenderer::renderMesh(ml::GraphicsDevice& g, const mat4f& cameraPerspective, const mat4f& meshToWorld,
                               const ml::D3D11TriMesh& mesh, const vec4f& color) {
  m_vsColor.bind();
  m_psColor.bind();

  AssetRendererConstantBuffer constants;

  constants.worldViewProj = cameraPerspective * meshToWorld;
  constants.modelColor = color;
  m_constants.updateAndBind(constants, 0);
  mesh.render();
}

void AssetRenderer::renderSphere(ml::GraphicsDevice& g, const mat4f& cameraPerspective, const vec3f& center,
                                 float radius, const vec4f& color) {
  renderMesh(g, cameraPerspective, mat4f::translation(center) * mat4f::scale(radius), m_sphere, color);
}

void AssetRenderer::renderCylinder(ml::GraphicsDevice& g, const mat4f& cameraPerspective, const vec3f& p0,
                                   const vec3f& p1, const vec4f& color) {
  renderMesh(g, cameraPerspective, mat4f::translation(p0) * mat4f::face(vec3f::eZ, p1 - p0) * mat4f::scale(1.0f, 1.0f,
             vec3f::dist(p0, p1)), m_cylinder, color);
}

void AssetRenderer::renderBox(ml::GraphicsDevice& g, const mat4f& cameraPerspective, const vec3f& center, float radius,
                              const vec4f& color) {
  renderMesh(g, cameraPerspective, mat4f::translation(center) * mat4f::scale(radius), m_box, color);
}

void AssetRenderer::renderPolygon(ml::GraphicsDevice& g, const mat4f& cameraPerspective, const ml::Polygonf& poly,
                                  const vec4f& color, float height) {
  for (const auto& segment : poly.segments()) {
    renderCylinder(g, cameraPerspective, vec3f(segment.p0(), height), vec3f(segment.p1(), height), color);
    renderSphere(g, cameraPerspective, vec3f(segment.p0(), height), 0.02f, color);
  }
}

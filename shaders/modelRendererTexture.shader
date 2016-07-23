
cbuffer ConstantBuffer : register( b0 )
{
	matrix worldViewProj;
	float4 modelColor;
}

Texture2D modelTexture : register( t0 );
SamplerState modelSampler : register( s0 );

struct VertexShaderOutput
{
    float4 position : SV_POSITION;
	float2 texCoord : TEXCOORD0;
	float3 normal : NORMAL;
	float3 worldPos : WORLDPOS;
};

VertexShaderOutput vertexShaderMain( float4 position : position,
									 float3 normal : normal,
									 float4 color : color,
									 float2 texCoord : texCoord )
{
    VertexShaderOutput output;
    output.position = mul( position, worldViewProj );
	output.texCoord = texCoord.xy;
	output.normal = normal;
	output.worldPos = position.xyz;
    return output;
}

float4 pixelShaderMain( VertexShaderOutput input ) : SV_Target
{
	float3 lightDir = normalize(float3(1.0, -2.0, 3.0));
	float3 lightColor = abs(dot(normalize(input.normal), lightDir)) * float3(0.6, 0.6, 0.6) + float3(0.4, 0.4, 0.4);
	float4 texColor = modelTexture.Sample(modelSampler, float2(input.texCoord.x, 1.0 - input.texCoord.y));
	
	return float4( modelColor.xyz * lightColor.xyz * texColor.xyz, 1.0f );
}

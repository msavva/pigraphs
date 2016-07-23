
cbuffer ConstantBuffer : register( b0 )
{
	matrix worldViewProj;
	float4 modelColor;
}

struct VertexShaderOutput
{
    float4 position : SV_POSITION;
	float4 color : TEXCOORD0;
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
	output.color = color;
	output.normal = normal;
	output.worldPos = position.xyz;
    return output;
}

float4 pixelShaderMain( VertexShaderOutput input ) : SV_Target
{
	float3 lightDir = normalize(float3(1.0, -2.0, 3.0));
	float3 lightColor = abs(dot(normalize(input.normal), lightDir)) * float3(0.7, 0.7, 0.7) + float3(0.3, 0.3, 0.3);
	return float4( lightColor * modelColor.xyz * input.color.xyz, 1.0f );
}

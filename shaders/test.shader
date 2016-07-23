cbuffer ConstantBuffer : register( b0 )
{
	matrix worldViewProj;
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
									 float4 attributeA : color,
									 float4 attributeB : texcoord)
{
    VertexShaderOutput output;
    output.position = mul( position, worldViewProj );
	output.color = attributeA;
	output.normal = normal;
	output.worldPos = position.xyz;
    return output;
}

float4 pixelShaderMain( VertexShaderOutput input ) : SV_Target
{
	float3 lightDir = normalize(float3(1.0, -2.0, 3.0));
	float l = abs(dot(normalize(input.normal), lightDir)) * 0.7 + 0.3;
    return float4(l * input.color.xyz, input.color.w);
	//return float4( input.color.xyzw );
}

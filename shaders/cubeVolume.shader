cbuffer ConstantBuffer : register( b0 )
{
	matrix worldViewProj;
	matrix worldViewProjInv;
	matrix worldToCubeTransform;
}

struct VertexShaderOutput
{
    float4 position : SV_POSITION;
	float4 color : TEXCOORD0;
	float3 normal : NORMAL;
	float3 worldPos : WORLDPOS;
};

Texture3D cubeTexture : register( t0 );
SamplerState cubeSampler : register( s0 );

float3 project(float4 v)
{
	return v.xyz / v.w;
}

float3 evaluatePoint(float3 pt)
{
	//float dist = length(pt - float3(0.5, 0.5, 0.5));
	//if(dist < 0.4)
	//	return float3(0.03, 0.0, 0.0);
	//return float3(0.0, 0.0, 0.0);

	if(pt.x <= 0.0 || pt.x >= 1.0 ||
	   pt.y <= 0.0 || pt.y >= 1.0 || 
	   pt.z <= 0.0 || pt.z >= 1.0)
	   return float3(0.0, 0.0, 0.0);

	return cubeTexture.Sample(cubeSampler, float3(pt)).xyz;
}

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
	float increment = 0.02;
	float sqrt3 = 1.732050808;
	int maxSteps = int(sqrt3 / increment) + 1;

	float3 projPos = project( mul( float4(input.worldPos, 1.0), worldViewProj ) );
	float3 offsetProjPos = projPos + float3(0.0, 0.0, 0.1);

	float3 offsetWorldPos = project( mul( float4(offsetProjPos, 1.0), worldViewProjInv ) );

	float3 cubePos = project( mul( float4(input.worldPos, 1.0), worldToCubeTransform ) );
	float3 cubePosDir = project( mul( float4(offsetWorldPos, 1.0), worldToCubeTransform ) ) - cubePos;

	float3 cubePosIncrement = cubePosDir * -increment / length(cubePosDir);

	float3 sum = float3(0.0, 0.0, 0.0);

	for(int i = 0; i < maxSteps; i++)
	{
		sum += evaluatePoint(cubePos);
		cubePos += cubePosIncrement;
	}

    return float4(sum, 0.5f);
}

// This class and its functions are derivatives of the work of UnrealCV, https://unrealcv.org/
// Licensed under the MIT License.


#include "AnnotationComponent.h"
// Overwrite the material

#include "Runtime/CoreUObject/Public/UObject/ConstructorHelpers.h"
#include "Runtime/Engine/Public/Materials/Material.h"
#include "Runtime/Engine/Public/Materials/MaterialInstanceDynamic.h"
#include "Runtime/Engine/Classes/Engine/StaticMesh.h"
#include "Runtime/Engine/Classes/Components/SkeletalMeshComponent.h"
#include "Runtime/Launch/Resources/Version.h"
#include "Runtime/Engine/Public/MaterialShared.h"
#include "Runtime/Engine/Classes/Engine/Engine.h"
#include "AirBlueprintLib.h"

#if ENGINE_MAJOR_VERSION >= 5
//different header files in UE
#include "Runtime/Engine/Public/StaticMeshSceneProxy.h"
#include "Runtime/Engine/Public/SkeletalMeshSceneProxy.h"

#endif
#include "Runtime/Engine/Public/Rendering/SkeletalMeshRenderData.h"

// For the procedural annotation proxy. ProceduralMeshComponent is already a public dependency of
// this module (AirSim.Build.cs), because the URDF pawn builds its links with it.
#include "ProceduralMeshComponent.h"
#include "DynamicMeshBuilder.h"
#include "StaticMeshResources.h"
#include "MaterialDomain.h"
// Only forward-declared by SceneManagement.h/PrimitiveSceneProxy.h; the definition is needed to
// construct one in GetDynamicMeshElements.
#include "PrimitiveUniformShaderParametersBuilder.h"
#include "Materials/MaterialRenderProxy.h"

/** A proxy class to get mesh data from StaticMesh, should be used together with AnnotationCamSensor.
Inheritance is needed because I need to access protected data
Use `show Material` command to see the effect of this component
Note that some area might be not colored, this is caused by the issue that
both the original mesh and the annotation mesh are rendered, this is not an issue for the AnnotationCamSensor, which will exclude original meshes.
*/
class FStaticAnnotationSceneProxy : public FStaticMeshSceneProxy
{
public:
	FMaterialRenderProxy* MaterialRenderProxy;

	//FStaticMeshSceneProxyDesc::InitializeFrom(UStaticMeshComponent* Component);

	FStaticAnnotationSceneProxy(UStaticMeshComponent* Component, bool bForceLODsShareStaticLighting, UMaterialInterface* AnnotationMID) :
		FStaticMeshSceneProxy(Component, bForceLODsShareStaticLighting)
	{
		MaterialRenderProxy = AnnotationMID->GetRenderProxy();
		// this->MaterialRelevance = AnnotationMID->GetRelevance(GetScene().GetFeatureLevel());
		// Note: This MaterailRelevance makes no difference?

		this->bVerifyUsedMaterials = false;
		// This is required, otherwise the code will fail

		bCastShadow = false;
	}

	virtual void GetDynamicMeshElements(
		const TArray < const FSceneView * > & Views,
		const FSceneViewFamily & ViewFamily,
		uint32 VisibilityMap,
		FMeshElementCollector & Collector) const override;

	virtual bool GetMeshElement
	(
		int32 LODIndex,
		int32 BatchIndex,
		int32 ElementIndex,
		uint8 InDepthPriorityGroup,
		bool bUseSelectedMaterial,
		bool bAllowPreCulledIndices,
		FMeshBatch & OutMeshBatch
	) const override;

	virtual FPrimitiveViewRelevance GetViewRelevance(const FSceneView * View) const override;
};

FPrimitiveViewRelevance FStaticAnnotationSceneProxy::GetViewRelevance(const FSceneView * View) const
{
	if (View->Family->EngineShowFlags.Materials)
	{
		FPrimitiveViewRelevance ViewRelevance;
		ViewRelevance.bDrawRelevance = 0; 
		// This will make the AnnotationComponent gets ignored if the Materials flag is on
		// Which means it won't affect regulary rendering.
		return ViewRelevance;
	}
	else
	{
		return FStaticMeshSceneProxy::GetViewRelevance(View);
	}
}


void FStaticAnnotationSceneProxy::GetDynamicMeshElements(
	const TArray < const FSceneView * > & Views,
	const FSceneViewFamily & ViewFamily,
	uint32 VisibilityMap,
	FMeshElementCollector & Collector) const
{
	//if (MaterialRenderProxy->GetMaterialName().Contains("AnnotationMaterialMID")) {
	//	FStaticMeshSceneProxy::GetDynamicMeshElements(Views, ViewFamily, VisibilityMap, Collector);
	//}	
	FStaticMeshSceneProxy::GetDynamicMeshElements(Views, ViewFamily, VisibilityMap, Collector);

}

bool FStaticAnnotationSceneProxy::GetMeshElement(
	int32 LODIndex,
	int32 BatchIndex,
	int32 ElementIndex,
	uint8 InDepthPriorityGroup,
	bool bUseSelectedMaterial,
	bool bAllowPreCulledIndices,
	FMeshBatch & OutMeshBatch) const
{
	bool Ret = FStaticMeshSceneProxy::GetMeshElement(LODIndex, BatchIndex, ElementIndex, InDepthPriorityGroup,
		bUseSelectedMaterial, bAllowPreCulledIndices, OutMeshBatch);
	OutMeshBatch.MaterialRenderProxy = this->MaterialRenderProxy;
	return Ret;
}

class FSkeletalAnnotationSceneProxy : public FSkeletalMeshSceneProxy
{
public:
	FSkeletalAnnotationSceneProxy(const USkinnedMeshComponent* Component, FSkeletalMeshRenderData* InSkeletalMeshRenderData, UMaterialInterface* AnnotationMID)
	: FSkeletalMeshSceneProxy(Component, InSkeletalMeshRenderData)
	{
		// TODO: Update MaterialRelevance
		this->bVerifyUsedMaterials = false;
		// this->bCastShadow = false;
		this->bCastDynamicShadow = false;
		for(int32 LODIdx=0; LODIdx < LODSections.Num(); LODIdx++)
		{
			FLODSectionElements& LODSection = LODSections[LODIdx];
			for(int32 SectionIndex = 0; SectionIndex < LODSection.SectionElements.Num(); SectionIndex++)
			{
				if (IsValid(AnnotationMID))
				{
					LODSection.SectionElements[SectionIndex].Material = AnnotationMID;
				}
				else
				{
					UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation: AnnotationMaterial is Invalid in FSkeletalSceneProxy"));
				}
			}
		}
	}
	virtual FPrimitiveViewRelevance GetViewRelevance(const FSceneView * View) const override;

	virtual void GetDynamicMeshElements(
		const TArray<const FSceneView*>& Views,
		const FSceneViewFamily& ViewFamily,
		uint32 VisibilityMap,
		FMeshElementCollector& Collector) const;
};


void FSkeletalAnnotationSceneProxy::GetDynamicMeshElements(
	const TArray<const FSceneView*>& Views,
	const FSceneViewFamily& ViewFamily,
	uint32 VisibilityMap,
	FMeshElementCollector& Collector) const
{
	//if (LODSections.Num() > 0){
	//	if (LODSections[0].SectionElements.Num() > 0) {
	//		if (LODSections[0].SectionElements[0].Material->GetName().Contains("AnnotationMaterialMID")) {
	//			FSkeletalMeshSceneProxy::GetDynamicMeshElements(Views, ViewFamily, VisibilityMap, Collector);
	//		}
	//	}
	//}
	FSkeletalMeshSceneProxy::GetDynamicMeshElements(Views, ViewFamily, VisibilityMap, Collector);

}

FPrimitiveViewRelevance FSkeletalAnnotationSceneProxy::GetViewRelevance(const FSceneView * View) const
{
	if (View->Family->EngineShowFlags.Materials)
	{
		FPrimitiveViewRelevance ViewRelevance;
		ViewRelevance.bDrawRelevance = 0; // This will make it gets ignored, when materials flag is enabled.
		return ViewRelevance;
	}
	else
	{
		return FSkeletalMeshSceneProxy::GetViewRelevance(View);
	}
}

/** Diagnostic only. Substitutes a known-good unlit engine material for the annotation MID on the
 *  procedural annotation path, to separate "the batch is wrong" from "the MID is wrong". */
static TAutoConsoleVariable<int32> CVarAnnotationProcDebugColor(
	TEXT("airsim.AnnotationProcDebugColor"),
	0,
	TEXT("Paint procedural annotation geometry with a flat engine material instead of the\n")
	TEXT("annotation MID.\n")
	TEXT("  0: off (default)\n")
	TEXT("  1: on - geometry renders bright green if the mesh batch itself is sound"),
	ECVF_Default);

/** One section of a procedural parent, mirrored into render resources this proxy owns.
 *
 * A copy rather than a reference on purpose: the parent's own proxy owns its buffers and is free to
 * rebuild them when the mesh changes, so sharing them would give two proxies one lifetime. The cost
 * is a duplicate vertex buffer per link, which for a 23-link robot is negligible against the render
 * target it is being drawn into. */
class FProcAnnotationProxySection
{
public:
	FStaticMeshVertexBuffers VertexBuffers;
	FDynamicMeshIndexBuffer32 IndexBuffer;
	FLocalVertexFactory VertexFactory;
	bool bSectionVisible;

	FProcAnnotationProxySection(ERHIFeatureLevel::Type InFeatureLevel)
		: VertexFactory(InFeatureLevel, "FProcAnnotationProxySection")
		, bSectionVisible(true)
	{}
};

/** Annotation proxy for a UProceduralMeshComponent parent.
 *
 * ⚠ This is the one of the three that does NOT subclass an engine proxy, and the reason is worth
 * recording because it looks like an oversight otherwise. FProceduralMeshSceneProxy is declared
 * `final` in ProceduralMeshComponent.cpp, a Private translation unit with no public header — so the
 * trick the static and skeletal proxies use (inherit, then swap the material in GetMeshElement /
 * the LOD sections) is not available. What IS public is the section data itself, through
 * UProceduralMeshComponent::GetNumSections() and GetProcMeshSection(), so this proxy rebuilds the
 * render resources from that and draws them with the annotation material directly.
 *
 * Behaviour is otherwise identical to the other two: invisible whenever the Materials show flag is
 * on (so it never disturbs normal rendering), no shadows, one flat unlit colour. */
class FProceduralAnnotationSceneProxy final : public FPrimitiveSceneProxy
{
public:
	SIZE_T GetTypeHash() const override
	{
		static size_t UniquePointer;
		return reinterpret_cast<size_t>(&UniquePointer);
	}

	FProceduralAnnotationSceneProxy(UPrimitiveComponent* AnnotationOwner,
									UProceduralMeshComponent* Parent,
									UMaterialInterface* AnnotationMID)
		: FPrimitiveSceneProxy(AnnotationOwner)
		, Material(AnnotationMID)
		, MaterialRelevance(AnnotationMID
								? AnnotationMID->GetRelevance(GetScene().GetFeatureLevel())
								: FMaterialRelevance())
	{
		if (Material == nullptr)
		{
			// Never leave it null: GetDynamicMeshElements dereferences it every frame, and a
			// missing annotation material must degrade to a visible wrong colour rather than crash.
			Material = UMaterial::GetDefaultMaterial(MD_Surface);
			UE_LOG(LogTemp, Warning,
				   TEXT("AirSim Annotation: AnnotationMaterial is Invalid in FProceduralAnnotationSceneProxy"));
		}

		// The annotation pass is unlit and flat, so it must not contribute shadows.
		bCastDynamicShadow = false;
		bVerifyUsedMaterials = false;

		const int32 NumSections = Parent->GetNumSections();
		Sections.AddZeroed(NumSections);
		for (int32 SectionIdx = 0; SectionIdx < NumSections; SectionIdx++)
		{
			FProcMeshSection* SrcSection = Parent->GetProcMeshSection(SectionIdx);
			if (SrcSection == nullptr || SrcSection->ProcIndexBuffer.Num() == 0 ||
				SrcSection->ProcVertexBuffer.Num() == 0)
			{
				continue;
			}

			FProcAnnotationProxySection* NewSection =
				new FProcAnnotationProxySection(GetScene().GetFeatureLevel());

			const int32 NumVerts = SrcSection->ProcVertexBuffer.Num();
			TArray<FDynamicMeshVertex> Vertices;
			Vertices.SetNumUninitialized(NumVerts);
			for (int32 VertIdx = 0; VertIdx < NumVerts; VertIdx++)
			{
				const FProcMeshVertex& ProcVert = SrcSection->ProcVertexBuffer[VertIdx];
				FDynamicMeshVertex& Vert = Vertices[VertIdx];
				Vert.Position = (FVector3f)ProcVert.Position;
				Vert.Color = ProcVert.Color;
				Vert.TextureCoordinate[0] = FVector2f(ProcVert.UV0);
				Vert.TextureCoordinate[1] = FVector2f(ProcVert.UV1);
				Vert.TextureCoordinate[2] = FVector2f(ProcVert.UV2);
				Vert.TextureCoordinate[3] = FVector2f(ProcVert.UV3);
				Vert.TangentX = ProcVert.Tangent.TangentX;
				Vert.TangentZ = ProcVert.Normal;
				Vert.TangentZ.Vector.W = ProcVert.Tangent.bFlipTangentY ? -127 : 127;
			}

			NewSection->IndexBuffer.Indices = SrcSection->ProcIndexBuffer;
			NewSection->VertexBuffers.InitFromDynamicVertex(&NewSection->VertexFactory, Vertices, 4);

			BeginInitResource(&NewSection->VertexBuffers.PositionVertexBuffer);
			BeginInitResource(&NewSection->VertexBuffers.StaticMeshVertexBuffer);
			BeginInitResource(&NewSection->VertexBuffers.ColorVertexBuffer);
			BeginInitResource(&NewSection->IndexBuffer);
			BeginInitResource(&NewSection->VertexFactory);

			NewSection->bSectionVisible = SrcSection->bSectionVisible;
			Sections[SectionIdx] = NewSection;
		}

		// ⚠ Diagnostic, deliberately unconditional and at Log level.
		//
		// The failure this exists to disambiguate: the robot's links register correctly, get
		// distinct segmentation IDs and colours, the annotator reports success — and not one pixel
		// appears. That is consistent with TWO opposite causes: the proxy is never constructed, or
		// it is constructed and then culled. Guessing between them has already cost several
		// rebuilds. If this line is absent from the log the answer is the former; if it is present
		// with a non-zero section count, the answer is the latter and the fault is in
		// GetViewRelevance or the ShowOnlyComponents whitelist.
		//
		// At Log, not Verbose: a Verbose diagnostic on this exact path was written once and never
		// printed, and its silence was misread as evidence.
		int32 built = 0;
		for (const FProcAnnotationProxySection* S : Sections)
			built += (S != nullptr) ? 1 : 0;
		// ⚠ Do NOT log GetBounds() here. A previous revision did, read zero, and reported it as the
		// cause. It is always zero at this point: FPrimitiveSceneProxy's constructor does not
		// receive bounds — FScene::AddPrimitive assigns them afterwards through SetTransform. The
		// reading was an artifact of where it was taken, not a property of the proxy.
		// ⚠ Read the colour back off the MID. The geometry now draws and occludes correctly but
		// renders BLACK, and there are two candidate causes: the MID genuinely holds black at the
		// moment this proxy is built (OnRegister creates a fresh MID seeded from AnnotationColor,
		// which is still default when PaintRGBComponent registers the component — the real colour
		// is only assigned afterwards), or the colour is right and the material is not producing it.
		FLinearColor ReadBack(ForceInit);
		bool bHasParam = false;
		if (UMaterialInstanceDynamic* MID = Cast<UMaterialInstanceDynamic>(Material))
			bHasParam = MID->GetVectorParameterValue(FMaterialParameterInfo(TEXT("AnnotationColor")), ReadBack);

		UE_LOG(LogTemp, Log,
			   TEXT("AirSim Annotation: procedural proxy built for '%s' - %d/%d sections, material '%s', "
					"AnnotationColor exists=%d value=(%.3f,%.3f,%.3f)"),
			   *Parent->GetName(), built, NumSections, *GetNameSafe(Material),
			   bHasParam ? 1 : 0, ReadBack.R, ReadBack.G, ReadBack.B);
	}

	virtual ~FProceduralAnnotationSceneProxy()
	{
		for (FProcAnnotationProxySection* Section : Sections)
		{
			if (Section != nullptr)
			{
				Section->VertexBuffers.PositionVertexBuffer.ReleaseResource();
				Section->VertexBuffers.StaticMeshVertexBuffer.ReleaseResource();
				Section->VertexBuffers.ColorVertexBuffer.ReleaseResource();
				Section->IndexBuffer.ReleaseResource();
				Section->VertexFactory.ReleaseResource();
				delete Section;
			}
		}
	}

	virtual void GetDynamicMeshElements(const TArray<const FSceneView*>& Views,
										const FSceneViewFamily& ViewFamily,
										uint32 VisibilityMap,
										FMeshElementCollector& Collector) const override
	{
		// One-shot: separates "never asked to draw" from "drew, but invisible". Those need
		// opposite fixes and reasoning alone has picked wrong twice.
		if (!bLoggedDraw)
		{
			bLoggedDraw = true;
			UE_LOG(LogTemp, Log,
				   TEXT("AirSim Annotation: procedural proxy DRAWING - %d views, visibility 0x%x, %d sections"),
				   Views.Num(), VisibilityMap, Sections.Num());
		}

		FMaterialRenderProxy* MaterialProxy = Material->GetRenderProxy();

		// ⚠ DIAGNOSTIC BISECTION, off by default: airsim.AnnotationProcDebugColor 1
		//
		// The procedural annotation geometry is relevant, collected, depth-correct and occludes the
		// scene properly, yet paints solid black — while the proxy's MID demonstrably holds the
		// right AnnotationColor and the annotation material is unlit (MSM_Unlit), so
		// ApplyViewModeOverrides leaves it untouched under Materials=false either way.
		//
		// Two possibilities remain and they need opposite fixes: the batch/vertex-factory setup is
		// wrong (nothing would shade), or the annotation MID specifically is not producing colour
		// through this path. Substituting a known-good engine material answers it in one run:
		// if the robot turns bright green, the batch is fine and the fault is the MID; if it stays
		// black, the fault is in this batch and the MID is irrelevant.
		static const auto* CVarDebugColor =
			IConsoleManager::Get().FindConsoleVariable(TEXT("airsim.AnnotationProcDebugColor"));
		if (CVarDebugColor != nullptr && CVarDebugColor->GetInt() != 0 &&
			GEngine != nullptr && GEngine->LevelColorationUnlitMaterial != nullptr)
		{
			FColoredMaterialRenderProxy* DebugProxy = new FColoredMaterialRenderProxy(
				GEngine->LevelColorationUnlitMaterial->GetRenderProxy(), FLinearColor(0.f, 1.f, 0.f));
			Collector.RegisterOneFrameMaterialProxy(DebugProxy);
			MaterialProxy = DebugProxy;
		}

		for (const FProcAnnotationProxySection* Section : Sections)
		{
			if (Section == nullptr || !Section->bSectionVisible)
				continue;

			for (int32 ViewIndex = 0; ViewIndex < Views.Num(); ViewIndex++)
			{
				if (!(VisibilityMap & (1 << ViewIndex)))
					continue;

				FMeshBatch& Mesh = Collector.AllocateMesh();
				FMeshBatchElement& BatchElement = Mesh.Elements[0];
				BatchElement.IndexBuffer = &Section->IndexBuffer;
				Mesh.bWireframe = false;
				Mesh.VertexFactory = &Section->VertexFactory;
				Mesh.MaterialRenderProxy = MaterialProxy;

				FDynamicPrimitiveUniformBuffer& DynamicPrimitiveUniformBuffer =
					Collector.AllocateOneFrameResource<FDynamicPrimitiveUniformBuffer>();
				FPrimitiveUniformShaderParametersBuilder Builder;
				BuildUniformShaderParameters(Builder);
				DynamicPrimitiveUniformBuffer.Set(Collector.GetRHICommandList(), Builder);
				BatchElement.PrimitiveUniformBufferResource = &DynamicPrimitiveUniformBuffer.UniformBuffer;

				BatchElement.FirstIndex = 0;
				BatchElement.NumPrimitives = Section->IndexBuffer.Indices.Num() / 3;
				BatchElement.MinVertexIndex = 0;
				BatchElement.MaxVertexIndex =
					Section->VertexBuffers.PositionVertexBuffer.GetNumVertices() - 1;
				Mesh.ReverseCulling = IsLocalToWorldDeterminantNegative();
				Mesh.Type = PT_TriangleList;
				Mesh.DepthPriorityGroup = SDPG_World;

				// ⚠ true, unlike the engine's own procedural proxy, which sets this false.
				//
				// The annotation capture renders with ShowFlags.Materials=false and Lighting=false.
				// With overrides disabled the batch keeps a material the renderer will not shade in
				// that configuration, and the result is geometry that occludes correctly and paints
				// solid black — measured: correct depth, correct occlusion of wall and ground, and
				// rgb(0,0,0) where the robot stands, while the live proxy's MID demonstrably held
				// the right AnnotationColor.
				Mesh.bCanApplyViewModeOverrides = true;
				Mesh.bUseWireframeSelectionColoring = false;
				Collector.AddMesh(ViewIndex, Mesh);
			}
		}
	}

	virtual FPrimitiveViewRelevance GetViewRelevance(const FSceneView* View) const override
	{
		// Same contract as FStaticAnnotationSceneProxy: stay out of any view that is drawing
		// materials, so the annotation geometry never doubles up on the Scene image.
		FPrimitiveViewRelevance Result;
		if (View->Family->EngineShowFlags.Materials)
		{
			Result.bDrawRelevance = 0;
			return Result;
		}

		Result.bDrawRelevance = IsShown(View);
		Result.bShadowRelevance = false;

		// ⚠ DYNAMIC, and this is settled by the engine source rather than by preference.
		// IsRichView() (PrimitiveDrawingUtils.cpp) returns true whenever EngineShowFlags.Materials
		// is off — which is exactly how the annotation capture is configured — and
		// FStaticMeshSceneProxy::GetViewRelevance then takes its bDynamicRelevance branch. So the
		// annotation proxies that DO work render dynamically in this view too. A static-draw
		// implementation was written on the opposite assumption and removed.
		Result.bDynamicRelevance = true;

		// ⚠ Forced true, NOT ShouldRenderInMainPass().
		//
		// ObjectAnnotator sets bRenderInMainPass=false on every annotation component, so
		// ShouldRenderInMainPass() returns false here. With it false the mesh batches are collected
		// — GetDynamicMeshElements runs, 23 draws, one section each — and then nothing is written,
		// because the base pass is what puts colour in the render target. Measured: relevance=23,
		// drawing=23, rover pixels=0, with only wall and ground colours where the robot stands.
		//
		// Why the static annotation proxy survives the same flag is NOT because it uses static draw
		// lists — it does not, in this view: IsRichView() is true whenever Materials is off, so
		// FStaticMeshSceneProxy::GetViewRelevance takes its dynamic branch too. Left unexplained
		// rather than guessed at.
		//
		// This proxy is only ever relevant to an annotation capture — GetViewRelevance above returns
		// bDrawRelevance=0 whenever Materials is on — so forcing the main pass here cannot leak
		// annotation geometry into a normal view.
		Result.bRenderInMainPass = true;
		Result.bUsesLightingChannels = GetLightingChannelMask() != GetDefaultLightingChannelMask();
		Result.bRenderCustomDepth = ShouldRenderCustomDepth();
		MaterialRelevance.SetPrimitiveViewRelevance(Result);

		if (!bLoggedRelevance)
		{
			bLoggedRelevance = true;
			UE_LOG(LogTemp, Log,
				   TEXT("AirSim Annotation: procedural relevance - shown=%d sceneCapture=%d mainPass=%d "
						"opaque=%d captureOnly=%d"),
				   Result.bDrawRelevance ? 1 : 0, View->bIsSceneCapture ? 1 : 0,
				   Result.bRenderInMainPass ? 1 : 0, Result.bOpaque ? 1 : 0,
				   IsVisibleInSceneCaptureOnly() ? 1 : 0);
		}
		return Result;
	}

	virtual bool CanBeOccluded() const override { return !MaterialRelevance.bDisableDepthTest; }

	virtual uint32 GetMemoryFootprint() const override { return sizeof(*this) + GetAllocatedSize(); }

private:
	TArray<FProcAnnotationProxySection*> Sections;
	UMaterialInterface* Material;
	FMaterialRelevance MaterialRelevance;

	// Both const-called from the render thread; one-shot diagnostics only.
	mutable bool bLoggedRelevance = false;
	mutable bool bLoggedDraw = false;
};

// FString MeterialPath = TEXT("MaterialInstanceConstant'/UnrealCV/AnnotationColor_Inst.AnnotationColor_Inst'");
// static ConstructorHelpers::FObjectFinder<UMaterialInstanceDynamic> AnnotationMaterialObject(*MaterialPath);
UAnnotationComponent::UAnnotationComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
	  // , ParentMeshInfo(nullptr)
{
	bSkeletalMesh = false;
	bTexture = false;

	FString MaterialPath = TEXT("Material'/AirSim/HUDAssets/AnnotationMaterial.AnnotationMaterial'");
	static ConstructorHelpers::FObjectFinder<UMaterial> AnnotationMaterialObject(*MaterialPath);
	if (AnnotationMaterialObject.Object == nullptr)
    {
        UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation: Annotation material is not valid."));
    }
    else
    {
        AnnotationMaterial = AnnotationMaterialObject.Object;
	}

	FString MaterialPathSphere = TEXT("Material'/AirSim/HUDAssets/AnnotationMaterialSphere.AnnotationMaterialSphere'");
	static ConstructorHelpers::FObjectFinder<UMaterial> SphereMaterialObject(*MaterialPathSphere);
	if (SphereMaterialObject.Object == nullptr)
	{
		UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation: Sphere annotation material is not valid."));
	}
	else
	{
		SphereMaterial = SphereMaterialObject.Object;
	}

	// ParentMeshInfo = MakeShareable(new FParentMeshInfo(nullptr));
	// This will be invalid until attached to a MeshComponent
	this->PrimaryComponentTick.bCanEverTick = true;
}

void UAnnotationComponent::OnRegister()
{
	Super::OnRegister();

	if (this->GetFName().ToString().Contains("annotation_sphere")) {
		AnnotationMID = UMaterialInstanceDynamic::Create(SphereMaterial, this, TEXT("AnnotationMaterialMID"));
		if (!IsValid(AnnotationMID))
		{
			UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation: SphereMaterial is not correctly initialized"));
			return;
		}
		FLinearColor LinearAnnotationColor = FLinearColor(0, 0, 0, 1.0);
		AnnotationMID->SetVectorParameterValue("AnnotationColor", LinearAnnotationColor);
	}
	else {
		// Note: This can not be placed in the constructor, MID means material instance dynamic
		AnnotationMID = UMaterialInstanceDynamic::Create(AnnotationMaterial, this, TEXT("AnnotationMaterialMID"));
		if (!IsValid(AnnotationMID))
		{
			UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation: ColorAnnotationMaterial is not correctly initialized"));
			return;
		}
		const float OneOver255 = 1.0f / 255.0f;
		FLinearColor LinearAnnotationColor = FLinearColor(
			this->AnnotationColor.R * OneOver255,
			this->AnnotationColor.G * OneOver255,
			this->AnnotationColor.B * OneOver255,
			1.0
		);
		AnnotationMID->SetVectorParameterValue("AnnotationColor", LinearAnnotationColor);
	}
}

/** 
 * Note: The "exposure compensation" in "PostProcessVolume3" in the RR map will destroy the color
 * Saturate the color to 1. This is a mysterious behavior after tedious debug.
 */
void UAnnotationComponent::SetAnnotationColor(FColor NewAnnotationColor)
{
	if (NewAnnotationColor.R == 27)NewAnnotationColor.R = 26;
	if (NewAnnotationColor.G == 27)NewAnnotationColor.G = 26;
	if (NewAnnotationColor.B == 27)NewAnnotationColor.B = 26;
	if (NewAnnotationColor.R == 32)NewAnnotationColor.R = 31;
	if (NewAnnotationColor.G == 32)NewAnnotationColor.G = 31;
	if (NewAnnotationColor.B == 32)NewAnnotationColor.B = 31;
	if (NewAnnotationColor.R == 35)NewAnnotationColor.R = 34;
	if (NewAnnotationColor.G == 35)NewAnnotationColor.G = 34;
	if (NewAnnotationColor.B == 35)NewAnnotationColor.B = 34;
	if (NewAnnotationColor.R == 41)NewAnnotationColor.R = 40;
	if (NewAnnotationColor.G == 41)NewAnnotationColor.G = 40;
	if (NewAnnotationColor.B == 41)NewAnnotationColor.B = 40;
	if (NewAnnotationColor.R == 44)NewAnnotationColor.R = 43;
	if (NewAnnotationColor.G == 44)NewAnnotationColor.G = 43;
	if (NewAnnotationColor.B == 44)NewAnnotationColor.B = 43;
	if (NewAnnotationColor.R == 49)NewAnnotationColor.R = 48;
	if (NewAnnotationColor.G == 49)NewAnnotationColor.G = 48;
	if (NewAnnotationColor.B == 49)NewAnnotationColor.B = 48;
	if (NewAnnotationColor.R == 51)NewAnnotationColor.R = 50;
	if (NewAnnotationColor.G == 51)NewAnnotationColor.G = 50;
	if (NewAnnotationColor.B == 51)NewAnnotationColor.B = 50;
	this->AnnotationColor = NewAnnotationColor;
	const float OneOver255 = 1.0f / 255.0f; // TODO: Check 255 or 256?

	FLinearColor LinearAnnotationColor = FLinearColor(
		AnnotationColor.R * OneOver255,
		AnnotationColor.G * OneOver255,
		AnnotationColor.B * OneOver255,
		1.0
	);

	if (IsValid(AnnotationMID))
	{
		AnnotationMID->SetVectorParameterValue("AnnotationColor", LinearAnnotationColor);
	}
}

void UAnnotationComponent::SetAnnotationTexture(FString NewAnnotationTexturePath)
{
    bTexture = true;
	AnnotationMID->SetScalarParameterValue("TextureEnabled", 1);
    this->AnnotationTexturePath = NewAnnotationTexturePath;
    TArray<FString> splitPath;
    NewAnnotationTexturePath.ParseIntoArray(splitPath, TEXT("/"), true);
    FString TextureFileName = splitPath.Last();
	FString FullPath = FString::Printf(TEXT("%s.%s"), *NewAnnotationTexturePath, *TextureFileName);
	UTexture* AnnotationTexture = LoadObject<UTexture>(NULL, *FullPath);

    if (AnnotationTexture != nullptr)
    {       
        if (IsValid(AnnotationMID))
        {
			AnnotationMID->SetTextureParameterValue("AnnotationTexture", AnnotationTexture);
		}else
		{
			UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation: Could not set annotation texture to %s cause something wrong with MID."), *FullPath);
		}
    }
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation: Could not set annotation texture to %s."), *FullPath);
	}
}

void UAnnotationComponent::SetAnnotationTexture(UTexture* NewAnnotationTexture)
{
	bTexture = true;
	AnnotationMID->SetScalarParameterValue("TextureEnabled", 1);
	TArray<FString> splitPath;
	NewAnnotationTexture->GetPathName().ParseIntoArray(splitPath, TEXT("."), true);
	FString TextureFilePath = splitPath[0];
	this->AnnotationTexturePath = TextureFilePath;
	if (IsValid(AnnotationMID))
	{
		AnnotationMID->SetTextureParameterValue("AnnotationTexture", NewAnnotationTexture);
	}
}

FColor UAnnotationComponent::GetAnnotationColor()
{
	return AnnotationColor;
}

FString UAnnotationComponent::GetAnnotationTexturePath()
{
	return AnnotationTexturePath;
}

FPrimitiveSceneProxy* UAnnotationComponent::CreateSceneProxy(UStaticMeshComponent* StaticMeshComponent)
{
	// FPrimitiveSceneProxy* PrimitiveSceneProxy = StaticMeshComponent->CreateSceneProxy();
	// FStaticMeshSceneProxy* StaticMeshSceneProxy = (FStaticMeshSceneProxy*)PrimitiveSceneProxy;
	UMaterialInterface* ProxyMaterial = AnnotationMID; // Material Instance Dynamic
	UStaticMesh* ParentStaticMesh = StaticMeshComponent->GetStaticMesh();
	if(ParentStaticMesh == NULL
		|| ParentStaticMesh->GetRenderData() == NULL
		|| ParentStaticMesh->GetRenderData()->LODResources.Num() == 0)
		// || StaticMesh->RenderData->LODResources[0].VertexBuffer.GetNumVertices() == 0)
	{
		// UE_LOG(LogTemp, Warning, TEXT("%s, ParentStaticMesh is invalid."), *StaticMeshComponent->GetName());
		return NULL;
	}

	// FPrimitiveSceneProxy* Proxy = ::new FStaticMeshSceneProxy(OwnerComponent, false);
	FPrimitiveSceneProxy* Proxy = ::new FStaticAnnotationSceneProxy(StaticMeshComponent, false, ProxyMaterial);
	return Proxy;
	// This is not recommended, but I know what I am doing.
}

// See https://github.com/EpicGames/UnrealEngine/blob/release/Engine/Source/Runtime/Engine/Private/Components/SkinnedMeshComponent.cpp:417
FPrimitiveSceneProxy* UAnnotationComponent::CreateSceneProxy(USkeletalMeshComponent* SkeletalMeshComponent)
{
	UMaterialInterface* ProxyMaterial = AnnotationMID; // Material Instance Dynamic

	ERHIFeatureLevel::Type SceneFeatureLevel = GetWorld()->GetFeatureLevel();

	// Ref: https://github.com/EpicGames/UnrealEngine/blob/4.19/Engine/Source/Runtime/Engine/Private/Components/SkinnedMeshComponent.cpp#L415
	FSkeletalMeshRenderData* SkelMeshRenderData = SkeletalMeshComponent->GetSkeletalMeshRenderData();

	// Only create a scene proxy for rendering if properly initialized
	if (SkelMeshRenderData &&
		SkelMeshRenderData->LODRenderData.IsValidIndex(SkeletalMeshComponent->GetPredictedLODLevel()) &&
		SkeletalMeshComponent->MeshObject) // The risk of using MeshObject
	{
		// Only create a scene proxy if the bone count being used is supported, or if we don't have a skeleton (this is the case with destructibles)
		// int32 MaxBonesPerChunk = SkelMeshResource->GetMaxBonesPerSection();
		// if (MaxBonesPerChunk <= GetFeatureLevelMaxNumberOfBones(SceneFeatureLevel))
		// {
		//	Result = ::new FSkeletalAnnotationSceneProxy(SkeletalMeshComponent, SkelMeshResource, AnnotationMID);
		// }
		// TODO: The SkeletalMeshComponent might need to be recreated
		return new FSkeletalAnnotationSceneProxy(SkeletalMeshComponent, SkelMeshRenderData, ProxyMaterial);
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation: The data of SkeletalMeshComponent %s is invalid."), *SkeletalMeshComponent->GetName());
		return nullptr;
	}
}

FPrimitiveSceneProxy* UAnnotationComponent::CreateSceneProxy(UProceduralMeshComponent* ProcMeshComponent)
{
	// A procedural mesh built with no sections yet is the normal state during construction, not an
	// error — the URDF pawn registers the component before filling it. Returning null here means
	// "nothing to draw yet"; MarkRenderStateDirty on the parent brings us back once it has data.
	if (ProcMeshComponent->GetNumSections() == 0)
	{
		UE_LOG(LogTemp, Log,
			   TEXT("AirSim Annotation: procedural parent '%s' has no sections yet - no proxy"),
			   *ProcMeshComponent->GetName());
		return nullptr;
	}

	return ::new FProceduralAnnotationSceneProxy(this, ProcMeshComponent, AnnotationMID);
}


// TODO: This needs to be involked when the ParentComponent refresh its render state, otherwise it will crash the engine
FPrimitiveSceneProxy* UAnnotationComponent::CreateSceneProxy()
{
	// UMaterialInstanceDynamic* AnnotationMID = UMaterialInstanceDynamic::Create(AnnotationMaterial, this);
	// FColor AnnotationColor = FColor::MakeRandomColor();
	// AnnotationMID->SetVectorParameterByIndex(0, AnnotationColor);

	USceneComponent* ParentComponent = this->GetAttachParent();
	// USceneComponent* ParentComponent = this->ParentMeshInfo->GetParentMeshComponent();

	if (!IsValid(ParentComponent))
	{
		UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation: Parent component is invalid."));
		return nullptr;
	}


	UStaticMeshComponent* StaticMeshComponent = Cast<UStaticMeshComponent>(ParentComponent);
	USkeletalMeshComponent* SkeletalMeshComponent = Cast<USkeletalMeshComponent>(ParentComponent);
	UProceduralMeshComponent* ProcMeshComponent = Cast<UProceduralMeshComponent>(ParentComponent);
	// UCableComponent* CableComponent = Cast<UCableComponent>(ParentComponent);
	if (IsValid(StaticMeshComponent))
	{
		return CreateSceneProxy(StaticMeshComponent);
	}
	else if (IsValid(SkeletalMeshComponent))
	{
		bSkeletalMesh = true;
		return CreateSceneProxy(SkeletalMeshComponent);
	}
	else if (IsValid(ProcMeshComponent))
	{
		return CreateSceneProxy(ProcMeshComponent);
	}
	// else if (IsValid(CableComponent))
	// {
	// 	return CreateSceneProxy(CableComponent);
	// }
	else
	{
		//UE_LOG(LogTemp, Warning, TEXT("AirSim Annotation: The type of ParentMeshComponent : %s can not be supported."), *ParentComponent->GetClass()->GetName());
		return nullptr;
	}
	// return nullptr;
}

FBoxSphereBounds UAnnotationComponent::CalcBounds(const FTransform & LocalToWorld) const
{
	// UMeshComponent* ParentMeshComponent = ParentMeshInfo->GetParentMeshComponent();
	// if (IsValid(ParentMeshComponent))
	// {
	// 	return ParentMeshComponent->CalcBounds(LocalToWorld);
	// }
	// else
	// {
	// 	FBoxSphereBounds DefaultBounds;
	// 	return DefaultBounds;
	// }

	USceneComponent* Parent = this->GetAttachParent();
	UStaticMeshComponent* StaticMeshComponent = Cast<UStaticMeshComponent>(Parent);
	if (IsValid(StaticMeshComponent))
	{
		return StaticMeshComponent->CalcBounds(LocalToWorld);
	}

	USkeletalMeshComponent* SkeletalMeshComponent = Cast<USkeletalMeshComponent>(Parent);
	if (IsValid(SkeletalMeshComponent))
	{
		return SkeletalMeshComponent->CalcBounds(LocalToWorld);
	}

	// ⚠ Must be here as well as in CreateSceneProxy. Bounds that do not cover the geometry get the
	// primitive culled before the proxy is ever asked to draw, which would look exactly like the
	// proxy not working — a silent zero-pixel failure of the kind this whole path already produced
	// three times through other causes.
	UProceduralMeshComponent* ProcMeshComponent = Cast<UProceduralMeshComponent>(Parent);
	if (IsValid(ProcMeshComponent))
	{
		const FBoxSphereBounds B = ProcMeshComponent->CalcBounds(LocalToWorld);
		static bool bLoggedProcBounds = false;
		if (!bLoggedProcBounds)
		{
			bLoggedProcBounds = true;
			UE_LOG(LogTemp, Log,
				   TEXT("AirSim Annotation: CalcBounds(procedural) '%s' -> extent=(%.1f,%.1f,%.1f) radius=%.1f"),
				   *ProcMeshComponent->GetName(), B.BoxExtent.X, B.BoxExtent.Y, B.BoxExtent.Z,
				   B.SphereRadius);
		}
		return B;
	}

	FBoxSphereBounds DefaultBounds;
	DefaultBounds.Origin = LocalToWorld.GetLocation();
	DefaultBounds.BoxExtent = FVector::ZeroVector;
	DefaultBounds.SphereRadius = 0.f;
	return DefaultBounds;
}

// Extra overhead for the game scene
void UAnnotationComponent::TickComponent(
	float DeltaTime,
	enum ELevelTick TickType,
	FActorComponentTickFunction * ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction); 

	if (bSkeletalMesh)
	{
		MarkRenderStateDirty(); // Without it will break the SkeletalMeshComponent
	}
	/*
	// if (ParentMeshInfo->RequiresUpdate()) 
	// TODO: This sometimes miss a required update, see OWIMap. Not sure why.
	// TODO: Per-frame update is certainly wasted.
	{
		// FIXME: Update the render proxy per frame will cause jittering on the material.
		ParentMeshInfo = MakeShareable(new FParentMeshInfo(this->GetAttachParent()));
	}
	*/
}


void UAnnotationComponent::ForceUpdate()
{
	this->MarkRenderStateDirty();
}

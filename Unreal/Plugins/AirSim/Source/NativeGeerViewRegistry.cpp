// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "NativeGeerViewRegistry.h"

#include "RenderingThread.h"
#include "RHICommandList.h"

namespace
{
    TMap<const FRenderTarget*, FAirSimNativeGeerView> GAirSimNativeGeerViews;
    uint64 GAirSimNativeGeerRegistrationSerial = 0;
}

void AirSimRegisterNativeGeerView(FRenderTarget* output_target,
                                  FString camera_name,
                                  FIntPoint output_extent,
                                  const FAirSimRaymapResourcePtr& raymap,
                                  bool central)
{
    if (output_target == nullptr || !raymap.IsValid()) {
        UE_LOG(LogTemp, Error,
               TEXT("[AirSim][NativeGEER][GateA] refused registration: camera=%s target=%p raymap=%s"),
               *camera_name, output_target, raymap.IsValid() ? TEXT("valid") : TEXT("invalid"));
        return;
    }

    ENQUEUE_RENDER_COMMAND(AirSimRegisterNativeGeerViewCommand)
    ([output_target, camera_name = MoveTemp(camera_name), output_extent, raymap, central]
     (FRHICommandListImmediate&) mutable {
        check(IsInRenderingThread());

        if (!raymap->ready || raymap->width != static_cast<uint32>(output_extent.X) ||
            raymap->height != static_cast<uint32>(output_extent.Y)) {
            UE_LOG(LogTemp, Error,
                   TEXT("[AirSim][NativeGEER][GateA] refused render-thread registration: ")
                   TEXT("camera=%s target=%p output=%dx%d raymap=%ux%u ready=%s"),
                   *camera_name, output_target, output_extent.X, output_extent.Y,
                   raymap->width, raymap->height, raymap->ready ? TEXT("true") : TEXT("false"));
            return;
        }

        FAirSimNativeGeerView view;
        view.camera_name = MoveTemp(camera_name);
        view.output_extent = output_extent;
        view.raymap = raymap;
        view.registration_serial = ++GAirSimNativeGeerRegistrationSerial;
        view.central = central;
        GAirSimNativeGeerViews.Add(output_target, view);

        UE_LOG(LogTemp, Log,
               TEXT("[AirSim][NativeGEER][GateA] registered camera=%s target=%p serial=%llu ")
               TEXT("output=%dx%d raymap=%ux%u ready=true central=%s"),
               *view.camera_name, output_target,
               static_cast<unsigned long long>(view.registration_serial),
               view.output_extent.X, view.output_extent.Y,
               view.raymap->width, view.raymap->height,
               view.central ? TEXT("true") : TEXT("false"));
    });
}

void AirSimUnregisterNativeGeerView(FRenderTarget* output_target)
{
    if (output_target == nullptr)
        return;

    ENQUEUE_RENDER_COMMAND(AirSimUnregisterNativeGeerViewCommand)
    ([output_target](FRHICommandListImmediate&) {
        check(IsInRenderingThread());
        if (const FAirSimNativeGeerView* view = GAirSimNativeGeerViews.Find(output_target)) {
            UE_LOG(LogTemp, Log,
                   TEXT("[AirSim][NativeGEER][GateA] unregistered camera=%s target=%p serial=%llu"),
                   *view->camera_name, output_target,
                   static_cast<unsigned long long>(view->registration_serial));
            GAirSimNativeGeerViews.Remove(output_target);
        }
    });
}

bool AirSimFindNativeGeerView_RenderThread(const FRenderTarget* output_target,
                                           FAirSimNativeGeerView& out_view)
{
    check(IsInRenderingThread());
    const FAirSimNativeGeerView* view = GAirSimNativeGeerViews.Find(output_target);
    if (view == nullptr)
        return false;

    out_view = *view;
    return true;
}

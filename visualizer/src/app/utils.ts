//==============================================================================
// utils.ts
//==============================================================================
//
// Source code of the 3D visualization developed as a part of the
// "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
//
// Author: Jakub Výmola (kuba.vymola@gmail.com)
// Date: 04/30/2023
//
//==============================================================================

import {
    Scene,
    Vector3,
    StandardMaterial,
    Texture,
    Color3,
    Engine,
    Mesh,
    CreateSphere,
    Angle,
} from '@babylonjs/core';

import { EARTH_RADIUS } from './constants';
import Entity from "./entity";
import OriginCamera from "./originCamera";

function createEntity(entityName: string, scene: Scene, camera: OriginCamera): Entity {
    const earthEntity: Entity = new Entity(entityName, scene);
    camera.add(earthEntity);

    return earthEntity;
}

function createCamera(scene: Scene, canvas: HTMLCanvasElement): OriginCamera {
    const camera: OriginCamera = new OriginCamera("camera", new Vector3(0, 0, 6378 * 1000), 3, scene);
    camera.inertia = 0;
    camera.speed = 3;
    camera.minZ = 0.01;
    camera.maxZ = 50000000;
    camera.fov = 1;
    camera.attachControl(canvas, true);

    canvas.addEventListener("wheel", function(e) {
        camera.speed = Math.min(100000000, Math.max(3, camera.speed -= e.deltaY * 5));
    });

    return camera;
}

function createEarth(scene: Scene, earthEntity: Entity): Mesh {
    const earthSphere: Mesh = CreateSphere("sphere", { diameter: 2 * EARTH_RADIUS - 1 });
    earthSphere.parent = earthEntity;
    
    earthSphere.scaling = new Vector3(1.0, -1.0, 1.0);
    earthEntity.doublepos = new Vector3(0, 0, 0);
    earthEntity.rotate(new Vector3(0, 1, 0), Angle.FromDegrees(90).radians())
    
    const planetMaterial = new StandardMaterial('planetMaterial', scene);
    planetMaterial.diffuseTexture = new Texture('../assets/earth_texture.jpg', scene);
    planetMaterial.emissiveColor = new Color3(1, 1, 1);
    planetMaterial.disableLighting = true;
    planetMaterial.alphaMode = Engine.ALPHA_COMBINE;
    planetMaterial.diffuseTexture.hasAlpha = true;
    planetMaterial.useAlphaFromDiffuseTexture = true;

    earthSphere.material = planetMaterial;
    earthSphere.alphaIndex = 5;

    return earthSphere;
}

export {
    createEntity,
    createCamera,
    createEarth,
};

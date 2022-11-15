import {
    Scene,
    Vector3,
    TransformNode,
} from '@babylonjs/core';

import OriginCamera from "./originCamera";

class Entity extends TransformNode {
    // you must use the doublepos property instead of position directly
    private _doublepos: Vector3 = new Vector3();
    public get doublepos() { return this._doublepos; }
    public set doublepos(pos: Vector3) { this._doublepos.copyFrom(pos); }

    constructor(name: string, scene: Scene) {
        super(name, scene);
    }

    // This is called automatically by OriginCamera
    public update(cam: OriginCamera): void {
        this.position = this.doublepos.subtract(cam.doubletgt);
    }
}

export default Entity;

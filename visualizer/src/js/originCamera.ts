import {
    Scene,
    Vector3,
    ArcRotateCamera,
} from '@babylonjs/core';

import Entity from "./entity";

class OriginCamera extends ArcRotateCamera {
    private _list: Array<Entity> = new Array<Entity>();

    // double precision position
    // you must use the doublepos to change its position, instead of position directly.
    private _doublepos: Vector3 = new Vector3();
    public get doublepos() { return this._doublepos; }
    public set doublepos(pos: Vector3) { this._doublepos.copyFrom(pos); }

    // double precision target
    // you must use the doubletgt to change it, instead of setTarget() directly.
    private _doubletgt: Vector3 = new Vector3();
    public get doubletgt() { return this._doubletgt; }
    public set doubletgt(tgt: Vector3) {
        this._doubletgt.copyFrom(tgt);
        // this.setTarget(this._doubletgt.subtract(this._doublepos));
    }

    // Constructor
    constructor(name: string, target: Vector3, radius: number, scene: Scene) {
        super(name, Math.PI / 4, Math.PI / 4, radius, new Vector3(0, 0, 0), scene);

        // this.doublepos = position;
        this.doubletgt = target;

        this.lowerRadiusLimit = 0.3;
        this.angularSensibilityX = 500;
        this.angularSensibilityY = 500;

        this._scene.onBeforeActiveMeshesEvaluationObservable.add(() => {
            // accumulate any movement on current frame
            // to the double precision position,
            // then clear the camera movement (move camera back to origin);
            // this would not be necessary if we moved the camera
            // ouselves from this class, but for now we're
            // leaving that responsibility for the original UniversalCamera,
            // so when it moves from origin, we must update our doublepos
            // and reset the UniversalCamera back to origin.

            // this.doublepos.addInPlace(this.position);
            // this.position.set(0, 0, 0);

            this.setTarget(new Vector3(0, 0, 0));

            // iterate through all registered Entities
            for (let i=0; i<this._list.length; i++)
            {
                // update the Entity
                this._list[i].update(this);
            }
        });
    }

    // Register an Entity
    add(entity: Entity): void {
        this._list.push(entity);
    }
}

export default OriginCamera;

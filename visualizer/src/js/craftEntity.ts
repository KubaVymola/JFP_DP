import { Quaternion, Scene, TransformNode, Vector3 } from "@babylonjs/core";
import Entity from "./entity";

const SPINNER_COEF = 1 / 10;

class CraftEntity extends TransformNode {
    posOffset: Vector3;
    // pivot: Vector3;
    axis: Vector3;
    angleProperty: string;
    angleOffset: number;
    spinnerProperty: string;
    maxRPM: number;
    spinnerAngle: number;
    spinnerSenseProperty: string;
    
    constructor(name: string, scene: Scene) {
        super(name, scene);

        this.spinnerAngle = 0;
    }

    updateFromSim(data, deltaT_ms: number) {
        if (data[this.spinnerProperty]) {
            const currentRPM = Number(data[this.spinnerProperty]);
            const sense = Number(data[this.spinnerSenseProperty]);
            this.spinnerAngle += (currentRPM / 60) * 2 * Math.PI * (deltaT_ms / 1000) * SPINNER_COEF * sense;
            const rotationQuat: Quaternion = Quaternion.RotationAxis(this.axis, this.spinnerAngle);
            this.rotationQuaternion = rotationQuat;
        }
    }
}

export default CraftEntity;

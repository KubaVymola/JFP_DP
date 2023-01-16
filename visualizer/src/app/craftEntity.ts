import { Quaternion, Scene, TransformNode, Vector3 } from "@babylonjs/core";
import { DEG_TO_RAD } from "./constants";
import Entity from "./entity";

const SPINNER_COEF = 1 / 30;

class CraftEntity extends TransformNode {
    // posOffset: Vector3;
    // pivot: Vector3;
    axis: Vector3;
    angleProperty: string;
    angleOffset: number;
    spinnerProperty: string;
    maxRPM: number;
    spinnerAngle: number;
    propSenseProperty: string;
    thrustProperty: string;
    enginePitchProperty: string;
    engineYawProperty: string;
    
    constructor(name: string, scene: Scene) {
        super(name, scene);

        this.spinnerAngle = 0;
    }

    updateFromSim(data: any, deltaT_s: number) {
        const rotationQuat: Quaternion = Quaternion.Identity();
        
        if (this.spinnerProperty && this.spinnerProperty in data) {
            const currentRPM = Number(data[this.spinnerProperty]);
            const sense = Number(data[this.propSenseProperty]);
            this.spinnerAngle += (currentRPM / 60) * 2 * Math.PI * (deltaT_s) * SPINNER_COEF * sense;

            if (Number.isNaN(this.spinnerAngle)) {
                console.log('spinner nan');
                this.spinnerAngle = 0;
            }

            rotationQuat.multiplyInPlace(Quaternion.RotationAxis(this.axis, this.spinnerAngle));
        }
        
        if (this.thrustProperty) {
            const thrustNormalized = Math.pow(data[this.thrustProperty] || 0, 1/3) / 5;
            
            this.scaling = new Vector3(
                thrustNormalized,
                thrustNormalized,
                thrustNormalized,
            );
        }
        

        if (this.engineYawProperty && data[this.engineYawProperty]) {
            rotationQuat.multiplyInPlace(
                Quaternion.RotationAxis(new Vector3(0, 0, 1), data[this.engineYawProperty]),
            );
        }

        if (this.enginePitchProperty && data[this.enginePitchProperty]) {
            rotationQuat.multiplyInPlace(
                Quaternion.RotationAxis(new Vector3(0, 1, 0), data[this.enginePitchProperty]),
            );
        }

        this.rotationQuaternion = rotationQuat;
    }
}

export default CraftEntity;

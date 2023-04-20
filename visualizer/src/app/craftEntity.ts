//==============================================================================
// craftEntity.ts
//==============================================================================
//
// Source code of the 3D visualization developed as a part of the
// "Control Units Interface for JSBSim Simulator" thesis by Jakub Výmola
//
// Author: Jakub Výmola (kuba.vymola@gmail.com)
// Date: 04/30/2023
//
//==============================================================================

import { Quaternion, Scene, TransformNode, Vector3 } from "@babylonjs/core";
import { DEG_TO_RAD } from "./constants";
import Entity from "./entity";

const SPINNER_COEF = 1 / 30;

class CraftEntity extends TransformNode {
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
        
        /**
         * Visualize propellers
         */
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
        
        /**
         * Visualize thrust by scaling the red, semi-transparent cone
         */
        if (this.thrustProperty) {
            const thrustNormalized = Math.pow(data[this.thrustProperty] || 0, 1/3) / 5;
            
            this.scaling = new Vector3(
                thrustNormalized,
                thrustNormalized,
                thrustNormalized,
            );
        }
        
        /**
         * Visualize thrust-vectoring in yaw
         */
        if (this.engineYawProperty && data[this.engineYawProperty]) {
            rotationQuat.multiplyInPlace(
                Quaternion.RotationAxis(new Vector3(0, 0, 1), data[this.engineYawProperty]),
            );
        }

        /**
         * Visualize thrust-vectoring in pitch
         */
        if (this.enginePitchProperty && data[this.enginePitchProperty]) {
            rotationQuat.multiplyInPlace(
                Quaternion.RotationAxis(new Vector3(0, 1, 0), data[this.enginePitchProperty]),
            );
        }

        this.rotationQuaternion = rotationQuat;
    }
}

export default CraftEntity;

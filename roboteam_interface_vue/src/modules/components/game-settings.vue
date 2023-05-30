<script setup lang="ts">

import {proto} from "../../generated/proto";
import {useAiController} from "../composables/ai-controller";

const aiController = useAiController();

</script>
<template>
    <div class="grid grid-cols-fluid-12 gap-x-8 gap-y-4">
        <div class="form-control">
            <span class="">General</span>
            <label class="label cursor-pointer gap-2">
                <span class="label-text">AutoRef</span>
                <input type="checkbox" v-model="aiController.useReferee.value" class="toggle toggle-primary"/>
            </label>
            <label class="label cursor-pointer gap-2">
                <span class="label-text">Ignore Invariants</span>
                <input type="checkbox" v-model="aiController.ignoreInvariants.value" class="toggle toggle-primary"/>
            </label>
        </div>

        <div class="form-control">
            <span class="">Receiver</span>
            <div class="form-control">
                <label class="label cursor-pointer">
                    <span class="label-text"><font-awesome-icon icon="fa-vr-cardboard"/> Simulator</span>
                    <input type="radio" class="radio"
                           :value="proto.GameSettings.RobotHubMode.SIMULATOR"
                           v-model="aiController.robotHubMode.value"
                    />
                </label>
            </div>
            <div class="form-control">
                <label class="label cursor-pointer">
                    <span class="label-text"><font-awesome-icon icon="fa-robot"/> Basestation</span>
                    <input type="radio" class="radio"
                           :value="proto.GameSettings.RobotHubMode.BASESTATION"
                           v-model="aiController.robotHubMode.value"
                    />
                </label>
            </div>
        </div>

        <div class="form-control">
            <span class="">Side</span>
            <div class="form-control">
                <label class="label cursor-pointer">
                    <span class="label-text"><font-awesome-icon icon="fa-arrow-left"/> Left</span>
                    <input type="radio" class="radio" v-model="aiController.isLeft.value" :value="true"/>
                </label>
            </div>
            <div class="form-control">
                <label class="label cursor-pointer">
                    <span class="label-text"><font-awesome-icon icon="fa-arrow-right"/> Right</span>
                    <input type="radio" class="radio" v-model="aiController.isLeft.value" :value="false"/>
                </label>
            </div>
        </div>

        <div class="form-control">
            <span class="">Team</span>
            <div class="form-control">
                <label class="label cursor-pointer">
                    <span class="label-text"><font-awesome-icon icon="fa-people-group"
                                                                class="text-warning"/> Yellow</span>
                    <input type="radio" class="radio radio-warning" v-model="aiController.isYellow.value" :value="true"

                    />
                </label>
            </div>
            <div class="form-control">
                <label class="label cursor-pointer">
                    <span class="label-text"><font-awesome-icon icon="fa-people-group"
                                                                class="text-primary"/> Blue</span>
                    <input type="radio" class="radio radio-primary" v-model="aiController.isYellow.value" :value="false"/>
                </label>
            </div>
        </div>
    </div>
</template>
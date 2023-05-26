<script setup lang="ts">

import {proto} from "../../generated/proto";
import {useAIDataStore} from "../stores/dataStores/ai-data-store";
import {emitter} from "../../services/ai-events";
import {computed} from "vue";

const aiData = useAIDataStore();


</script>
<template>
    <div class="grid grid-cols-fluid-12 gap-x-8 gap-y-4">
        <div class="form-control">
            <span class="">General</span>
            <label class="label cursor-pointer gap-2">
                <span class="label-text">AutoRef</span>
                <input type="checkbox" v-model="aiData.useReferee" class="toggle toggle-primary"/>
            </label>
            <label class="label cursor-pointer gap-2">
                <span class="label-text">Ignore Invariants</span>
                <!--        <input type="checkbox" v-model="state?.runtimeConfig?.ignoreInvariants" class="toggle toggle-primary" checked/>-->
                <input type="checkbox" v-model="aiData.ignoreInvariants" class="toggle toggle-primary"/>
            </label>
        </div>

        <div class="form-control">
            <span class="">Receiver</span>
            <div class="form-control">
                <label class="label cursor-pointer">
                    <span class="label-text"><font-awesome-icon icon="fa-vr-cardboard"/> Simulator</span>
                    <input type="radio" class="radio"
                           :value="proto.GameSettings.RobotHubMode.SIMULATOR"
                           v-model="aiData.robotHubMode"
                    />
                </label>
            </div>
            <div class="form-control">
                <label class="label cursor-pointer">
                    <span class="label-text"><font-awesome-icon icon="fa-robot"/> Basestation</span>
                    <input type="radio" class="radio"
                           :value="proto.GameSettings.RobotHubMode.BASESTATION"
                           v-model="aiData.robotHubMode"
                    />
                </label>
            </div>
        </div>

        <div class="form-control">
            <span class="">Side</span>
            <div class="form-control">
                <label class="label cursor-pointer">
                    <span class="label-text"><font-awesome-icon icon="fa-arrow-left"/> Left</span>
                    <input type="radio" class="radio" v-model="aiData.isLeft" :value="true"/>
                </label>
            </div>
            <div class="form-control">
                <label class="label cursor-pointer">
                    <span class="label-text"><font-awesome-icon icon="fa-arrow-right"/> Right</span>
                    <input type="radio" class="radio" v-model="aiData.isLeft" :value="false"/>
                </label>
            </div>
        </div>

        <div class="form-control">
            <span class="">Team</span>
            <div class="form-control">
                <label class="label cursor-pointer">
                    <span class="label-text"><font-awesome-icon icon="fa-people-group"
                                                                class="text-warning"/> Yellow</span>
                    <input type="radio" class="radio radio-warning" v-model="aiData.isYellow" :value="true"

                    />
                </label>
            </div>
            <div class="form-control">
                <label class="label cursor-pointer">
                    <span class="label-text"><font-awesome-icon icon="fa-people-group"
                                                                class="text-primary"/> Blue</span>
                    <input type="radio" class="radio radio-primary" v-model="aiData.isYellow" :value="false"/>
                </label>
            </div>
        </div>
    </div>
</template>
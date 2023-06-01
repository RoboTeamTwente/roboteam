<script setup lang="ts">
import {Tab, TABS, useUIStore} from "../../stores/ui-store";
import TriState from "./tri-state.vue";
import {computed} from "vue";

const uiStore = useUIStore();

// This makes sure the UI Settings tab is always in the left panel
const leftPanelSelectedTabs = computed({
    get: () => uiStore.leftPanel.selectableTabs,
    set: (val: Tab[]) => {
        val = val.filter(e => e.name !== 'UI Settings');
        val.push(...TABS.filter(e => e.name === 'UI Settings'));
        uiStore.leftPanel.selectableTabs = val
    }
})

</script>
<template>
    <div class="grid grid-cols-fluid-12 gap-x-8 gap-y-4">
        <div class="form-control">
            <span class="">General</span>
            <label class="label cursor-pointer gap-2">
                <span class="label-text">Ball scale</span>
                <input type="number" v-model="uiStore.scaling.ball" min="1" step="0.25"
                       class="input input-sm input-bordered">
            </label>

            <label class="label cursor-pointer gap-2">
                <span class="label-text">Robots scale</span>
                <input type="number" v-model="uiStore.scaling.robots" min="1" step="0.25"
                       class="input input-sm input-bordered">
            </label>

            <span class="label-text">Internal Team</span>
            <div class="form-control">
                <label class="label cursor-pointer">
                    <span class="label-text"> Purple</span>
                    <input type="radio" class="radio radio-sm" v-model="uiStore.internalTeam" value="PURPLE"/>
                </label>
            </div>
            <div class="form-control">
                <label class="label cursor-pointer">
                    <span class="label-text">Black </span>
                    <input type="radio" class="radio radio-sm" v-model="uiStore.internalTeam" value="BLACK"/>
                </label>
            </div>
            <div class="form-control">
                <label class="label">
                    <span class="label-text">Select left bar tabs</span>
                </label>
                <select class="select select-bordered" multiple v-model="leftPanelSelectedTabs">
                    <template v-for="tab in TABS">
                        <option v-if="tab.name !== 'UI Settings'" :value="tab">{{ tab.name }}</option>
                        <option v-else :value="tab" disabled :selected="true">{{ tab.name }}</option>
                    </template>
                </select>
            </div>

            <div class="form-control">
                <label class="label">
                    <span class="label-text">Select bottom bar tabs</span>
                </label>
                <select class="select select-bordered" multiple v-model="uiStore.bottomPanel.selectableTabs">
                    <option v-for="tab in TABS" :value="tab">{{ tab.name }}</option>
                </select>
            </div>


            <div class="form-control mb-4">
                Velocities
                <tri-state v-model="uiStore.visualizations.velocities"/>
            </div>

            <div class="form-control mb-4">
                Path planning
                <tri-state v-model="uiStore.visualizations.pathPlanning"/>
            </div>

            <div class="form-control mb-4">
                Debug
                <tri-state v-model="uiStore.visualizations.debug"/>
            </div>
        </div>

    </div>
</template>
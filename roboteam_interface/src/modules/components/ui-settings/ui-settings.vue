<script setup lang="ts">
import { useUIStore } from '../../stores/ui-store'
import TriState from './tri-state.vue'
import DualState from './dual-state.vue'
import { computed } from 'vue'
import { TabKey, TABS_DEFINITION } from '../../../tabs'

const uiStore = useUIStore()

// This makes sure the UI Settings tab is always in the left panel
const leftPanelSelectedTabs = computed({
  get: () => uiStore.leftPanel.selectableTabs,
  set: (val: TabKey[]) => {
    val = val.filter((e) => e !== 'UI Settings')
    val.push('UI Settings')
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
        <input
          type="number"
          v-model="uiStore.scaling.ball"
          min="1"
          step="0.25"
          class="input input-sm input-bordered"
        />
      </label>

      <label class="label cursor-pointer gap-2">
        <span class="label-text">Robots scale</span>
        <input
          type="number"
          v-model="uiStore.scaling.robots"
          min="1"
          step="0.25"
          class="input input-sm input-bordered"
        />
      </label>

      <span class="label-text">Internal Team</span>
      <div class="form-control">
        <label class="label cursor-pointer">
          <span class="label-text"> Purple</span>
          <input
            type="radio"
            class="radio radio-sm"
            v-model="uiStore.internalTeam"
            value="PURPLE"
          />
        </label>
      </div>
      <div class="form-control">
        <label class="label cursor-pointer">
          <span class="label-text">Black </span>
          <input type="radio" class="radio radio-sm" v-model="uiStore.internalTeam" value="BLACK" />
        </label>
      </div>
      <div class="form-control">
        <label class="label">
          <span class="label-text">Select left bar tabs</span>
        </label>
        <select class="select select-bordered" multiple v-model="leftPanelSelectedTabs">
          <template v-for="tab in Object.keys(TABS_DEFINITION)" :key="tab">
            <option :value="tab" :disabled="tab === 'UI Settings'">{{ tab }}</option>
          </template>
        </select>
      </div>

      <div class="form-control">
        <label class="label">
          <span class="label-text">Select bottom bar tabs</span>
        </label>
        <select
          class="select select-bordered"
          multiple
          v-model="uiStore.bottomPanel.selectableTabs"
        >
          <option v-for="tab in Object.keys(TABS_DEFINITION)" :value="tab" :key="tab">
            {{ tab }}
          </option>
        </select>
      </div>

      <div class="form-control mb-4">
        Velocities
        <tri-state v-model="uiStore.visualizations.velocities" />
      </div>

      <div class="form-control mb-4">
        Path planning
        <tri-state v-model="uiStore.visualizations.pathPlanning" />
      </div>

      <div class="form-control mb-4">
        Debug
        <tri-state v-model="uiStore.visualizations.debug" />
      </div>

      <div class="form-control mb-4">
        Margins
        <dual-state v-model="uiStore.visualizations.margins" />
      </div>

      <div class="form-control mb-4">
        Robot Roles
        <dual-state v-model="uiStore.visualizations.robotroles" />
      </div>
    </div>
  </div>
</template>

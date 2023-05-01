<script setup lang="ts">
import {useUIStore} from "../stores/ui-store";
import TriState from "./tri-state.vue";
const uiStore = useUIStore();

</script>
<template>
  <div class="grid grid-cols-fluid-12 gap-x-8 gap-y-4">
    <div class="form-control">
      <span class="">General</span>
      <label class="label cursor-pointer gap-2">
        <span class="label-text">Ball scale</span>
        <input type="number" v-model="uiStore.scaling.ball" min="1" step="0.25" class="input input-sm input-bordered">
      </label>

      <label class="label cursor-pointer gap-2">
        <span class="label-text">Robots scale</span>
        <input type="number" v-model="uiStore.scaling.robots" min="1" step="0.25" class="input input-sm input-bordered">
      </label>
<!--      <label class="label cursor-pointer gap-2">-->
<!--        <span class="label-text">Show velocities</span>-->
<!--        <input type="checkbox" v-model="uiStore.showVelocity" class="toggle toggle-primary" checked/>-->
<!--      </label>-->

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
    </div>

  </div>
</template>
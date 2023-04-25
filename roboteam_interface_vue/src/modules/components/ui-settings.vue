<script setup lang="ts">
import {useUIStore} from "../stores/ui-store";
const uiStore = useUIStore();

</script>
<template>
  <div class="grid grid-cols-fluid-12 gap-x-8 gap-y-4">
    <div class="form-control">
      <span class="">General</span>
      <label class="label cursor-pointer gap-2">
        <span class="label-text">Ball scale</span>
        <input type="number" v-model="uiStore.scaling.ball" min="1" step="0.25" class="input input-sm input-bordered w-full max-w-xs">
      </label>

      <label class="label cursor-pointer gap-2">
        <span class="label-text">Robots scale</span>
        <input type="number" v-model="uiStore.scaling.robots" min="1" step="0.25" class="input input-sm input-bordered w-full max-w-xs">
      </label>

      <label class="label cursor-pointer gap-2">
        <span class="label-text">Show velocities</span>
        <input type="checkbox" v-model="uiStore.showVelocity" class="toggle toggle-primary" checked/>
      </label>

      <label class="label cursor-pointer gap-2">
        <span class="label-text">Show drawings</span>
        <input type="checkbox" v-model="uiStore.showDrawings" class="toggle toggle-primary" checked/>
      </label>
    </div>

  </div>
</template>
<script setup lang="ts">
import {useVisualizationStore} from "../stores/visualization-store";

const visualizationStore = useVisualizationStore();

</script>

<template>
  <div class="grid grid-cols-fluid-10 gap-2 auto-cols-max">
    <template v-for="[label, metric] in visualizationStore.$state.metrics" :key="label">
      <div class="bg-base-200 p-2 rounded-xl border border-base-300 w-min">
        <div class="flex flex-row items-center gap-2" v-if="metric.boundedValue != null">
          <div class="text-sm text-gray-500">{{ label }}</div>
          <progress class="progress w-56" :value="metric.boundedValue.value! - metric.boundedValue.min!" :max="metric.boundedValue.max! - metric.boundedValue.min!"></progress>
          <div class="text-sm text-gray-500">{{ metric.boundedValue.value }}</div>
        </div>

        <template v-if="metric.decimal != null">
          <div class="stat-title">{{ label }}</div>
          <div class="stat-value font-mono">{{ metric.decimal.value }} {{ metric.decimal.unit }}</div>
          <div class="stat-desc font-mono">{{ metric.decimal.minRecorded }} - {{metric.decimal.maxRecorded }}</div>
        </template>
      </div>
<!--      <div class="stats shadow w-min" v-if="metric.decimal != null">-->
<!--      <div class="stat">-->
<!--        <div class="stat-title">{{ label }}</div>-->
<!--        <div class="stat-value">{{ metric.decimal }}</div>-->
<!--      </div>-->
<!--      </div>-->
    </template>
  </div>
</template>

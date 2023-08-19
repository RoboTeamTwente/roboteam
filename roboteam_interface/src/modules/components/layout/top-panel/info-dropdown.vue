<script setup lang="ts">
const props = defineProps<{
    currentTick: number,
    tickDuration: number,
    avgTickDuration: number,
  }>(),
  emit = defineEmits<{
    (event: 'disconnect-from-ai'): void
  }>()
</script>

<template>
  <div class="dropdown dropdown-end">
    <label tabindex="0" class="btn btn-circle btn-ghost btn-sm">
      <font-awesome-icon icon="fa-info-circle" />
    </label>
    <div
      tabindex="0"
      class="card compact dropdown-content shadow-lg bg-base-100 rounded-box w-96 border dark:border-base-300"
    >
      <div class="card-body">
        <button class="btn btn-error btn-sm" @click="() => emit('disconnect-from-ai')">
          Disconnect from AI
        </button>
        <h2 class="card-title">Stats</h2>
        <ul>
          <li class='mb-2'><font-awesome-icon icon='hashtag' /> Tick number: {{ props.currentTick }}</li>
          <li class='mb-2'><font-awesome-icon icon='stopwatch' /> Tick duration: {{ props.tickDuration.toFixed(2) }}ms</li>
          <li class='mb-2'><font-awesome-icon icon='angle-double-right' /> Avg. duration: {{ props.avgTickDuration.toFixed(2) }}ms</li>
        </ul>

        <h2 class="card-title">Shortcuts</h2>
        <ul class="">
          <li>
            Place ball:
            <div class="kbd kbd-sm font-mono">shift</div>
            +
            <div class="kbd kbd-sm font-mono">right click</div>
          </li>
          <li>
            Place selected robots:
            <div class="kbd kbd-sm font-mono">alt</div>
            +
            <div class="kbd kbd-sm font-mono">right click</div>
          </li>
        </ul>
      </div>
    </div>
  </div>
</template>

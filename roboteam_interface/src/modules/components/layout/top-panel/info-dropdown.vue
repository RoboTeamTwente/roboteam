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
  <div class="dropdown dropdown-end relative">
    <label tabindex="0" class="btn btn-circle btn-ghost btn-sm">
      <font-awesome-icon icon="fa-info-circle" />
    </label>
    <div
      tabindex="0"
      class="card compact dropdown-content shadow-lg bg-base-100 rounded-box w-96 border dark:border-base-300 z-50"
    >
      <div class="card-body">
        <button class="btn btn-error btn-sm" @click="() => emit('disconnect-from-ai')">
          Disconnect from AI
        </button>
        <h2 class="card-title">Stats</h2>
        <ul>
          <li class='mb-2'><font-awesome-icon icon='hashtag' /> Tick number: {{ props.currentTick }}</li>
          <li class='mb-2'><font-awesome-icon icon='stopwatch' /> Tick duration: <span class='font-mono'>{{ props.tickDuration.toFixed(2) }}ms</span></li>
          <li class='mb-2'><font-awesome-icon icon='angle-double-right' /> Avg. duration: <span class='font-mono'>{{ props.avgTickDuration.toFixed(2) }}ms</span></li>
        </ul>

        <h2 class="card-title">Shortcuts</h2>
        <ul class="flex flex-col gap-2">
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
          <li>
            Zoom in/out:
            <div class="kbd kbd-sm font-mono">ctrl</div>
            +
            <div class="kbd kbd-sm font-mono">wheel</div>
          </li>

          <li>
            Move camera:
            <div class="kbd kbd-sm font-mono">ctrl</div>
            +
            <div class="kbd kbd-sm font-mono">right click</div>
            +
            <div class="kbd kbd-sm font-mono">drag mouse</div>
          </li>
        </ul>
      </div>
    </div>
  </div>
</template>

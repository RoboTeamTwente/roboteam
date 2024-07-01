<script setup lang="ts">
const props = defineProps<{
    currentTick: number
    tickDuration: number
    avgTickDuration: number
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
          <li class="mb-2">
            <font-awesome-icon icon="hashtag" /> Tick number: {{ props.currentTick }}
          </li>
          <li class="mb-2">
            <font-awesome-icon icon="stopwatch" /> Tick duration:
            <span class="font-mono">{{ props.tickDuration.toFixed(2) }}ms</span>
          </li>
          <li class="mb-2">
            <font-awesome-icon icon="angle-double-right" /> Avg. duration:
            <span class="font-mono">{{ props.avgTickDuration.toFixed(2) }}ms</span>
          </li>
        </ul>

        <h2 class="card-title">Shortcuts</h2>
        <ul class="flex flex-col gap-2">
          <li>
            Place ball:
            <div class="kbd kbd-sm font-mono">b / shift</div>
            +
            <div class="kbd kbd-sm font-mono">left click / click and drag</div>
          </li>
          <li>
            Shoot to mouse with 6 m/s:
            <div class="kbd kbd-sm font-mono">s</div>
            +
            <div class="kbd kbd-sm font-mono">left click</div>
          </li>
          <li>
            Shoot to mouse with [1-9] m/s:
            <div class="kbd kbd-sm font-mono">[1-9]</div>
            +
            <div class="kbd kbd-sm font-mono">left click</div>
          </li>
          <li>
            Place selected robots:
            <div class="kbd kbd-rsm font-mono">r / alt</div>
            +
            <div class="kbd kbd-sm font-mono">left click / click and drag</div>
          </li>
          <li>
            Zoom in/out:
            <div class="kbd kbd-sm font-mono">scroll wheel</div>
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

        <h2 class="card-title">Colour explanation</h2>
        <ul class="flex flex-col gap-2">

          <li>
            <div class="kbd kbd-sm font-mono" :style="{ backgroundColor: '#8a0000', color: '#ffffff' }">Harasser</div>
          </li>

          <li>
            <div class="kbd kbd-sm font-mono" :style="{ backgroundColor: '#FF6600', color: '#000000' }">Passer/striker/ball placer/free kick taker/penalty taker/kick off taker</div>
          </li>

          <li>
            <div class="kbd kbd-sm font-mono" :style="{ backgroundColor: '#ff00ff', color: '#ffffff' }">Receiver</div>
          </li>

          <li>
            <div class="kbd kbd-sm font-mono" :style="{ backgroundColor: '#000000', color: '#ffffff' }">Robot with card</div>
          </li>
          </ul>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
const props = defineProps<{
    currentTick: number
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
      class="card compact dropdown-content shadow-lg bg-base-100 rounded-box w-96 border"
    >
      <div class="card-body">
        <button class="btn btn-error btn-sm" @click="() => emit('disconnect-from-ai')">
          Disconnect from AI
        </button>
        <h2 class="card-title">Stats</h2>
        <ul class="font-mono">
          <li>Current tick: {{ props.currentTick }}</li>
        </ul>

        <h2 class="card-title">Shortcuts</h2>
        <ul class="font-mono">
          <li>
            Place ball:
            <div class="kbd kbd-sm">shift</div>
            +
            <div class="kbd kbd-sm">right click</div>
          </li>
        </ul>
      </div>
    </div>
  </div>
</template>

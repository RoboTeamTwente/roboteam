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
    <div tabindex="0" class="card compact dropdown-content shadow bg-base-100 rounded-box w-64">
      <div class="card-body">
        <button class="btn btn-error btn-sm" @click="() => emit('disconnect-from-ai')">
          Disconnect from AI
        </button>
        <h2 class="card-title">Performance Info</h2>
        <ul class="font-mono">
          <li>Tick: {{ props.currentTick }}</li>
        </ul>
      </div>
    </div>
  </div>
</template>

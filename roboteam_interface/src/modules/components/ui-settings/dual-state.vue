<script setup lang="ts">
import { DualState } from '../../stores/ui-store'
import { computed } from 'vue'

const props = defineProps<{
  modelValue: DualState
}>()

const emit = defineEmits<{
  (event: 'update:modelValue', payload: string): void
}>()

const value = computed({
  get: () => props.modelValue,
  set: (val: DualState) => emit('update:modelValue', val)
})
</script>

<template>
  <div class="form-control">
    <label class="label cursor-pointer gap-2">
      <span class="label-text">Hide</span>
      <input type="radio" class="radio radio-sm" v-model="value" value="HIDE" />
    </label>
    <label class="label cursor-pointer gap-2">
      <span class="label-text">Show</span>
      <input type="radio" class="radio radio-sm" v-model="value" value="SHOW" />
    </label>
  </div>
</template>

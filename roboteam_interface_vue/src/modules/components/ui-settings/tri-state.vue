<script setup lang="ts">
import { TriState } from '../../stores/ui-store'
import { computed } from 'vue'

const props = defineProps<{
  modelValue: TriState
}>()

const emit = defineEmits<{
  (event: 'update:modelValue', payload: string): void
}>()

const value = computed({
  get: () => props.modelValue,
  set: (val: TriState) => emit('update:modelValue', val)
})
</script>

<template>
  <div class="form-control">
    <label class="label cursor-pointer gap-2">
      <span class="label-text">Hide</span>
      <input type="radio" class="radio radio-sm" v-model="value" value="HIDE" />
    </label>
    <label class="label cursor-pointer gap-2">
      <span class="label-text">Selected robots</span>
      <input type="radio" class="radio radio-sm" v-model="value" value="FOR_SELECTED_ROBOTS" />
    </label>
    <label class="label cursor-pointer gap-2">
      <span class="label-text">Show</span>
      <input type="radio" class="radio radio-sm" v-model="value" value="SHOW" />
    </label>
  </div>
</template>

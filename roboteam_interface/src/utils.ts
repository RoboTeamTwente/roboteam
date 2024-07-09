import { ShallowRef } from 'vue'

export type NoUndefinedField<T> = { [P in keyof T]-?: NoUndefinedField<NonNullable<T[P]>> }
export type DeepReadonly<T> = T extends Function
  ? T
  : T extends object
  ? { readonly [K in keyof T]: DeepReadonly<T[K]> }
  : T
export type ShallowReadonlyRef<T> = ShallowRef<DeepReadonly<T>>

export const sleep = (time: number) => {
  return new Promise((resolve) => setTimeout(resolve, time))
}

export const OUT_OF_CANVAS_COORDINATES = {
  x: 100,
  y: 100,
  yaw: 0
}

export const robotNameMap = (team: 'BLACK' | 'PURPLE', id: number) => {
  if (team === 'PURPLE') {
    return {
      0: 'Vissues',
      1: 'Wall-E',
      2: 'Bob',
      3: 'Pumba',
      4: 'Ted',
      5: 'Eve',
      6: 'Susie',
      7: 'James',
      8: 'Lizzy',
      9: 'McQueen',
      10: 'Kevin',
      11: 'Brum',
      12: 'Van Robogh',
      13: 'Wout',
      14: 'Jenny',
      15: 'Hermann'
    }[id]
  }

  return ''
}

export const formatFloat = (pos?: number | null): string => {
  if (pos === undefined || pos === null) {
    return '0.00'
  }

  const value = Math.abs(pos).toFixed(2)
  const sign = pos > -0.0001 ? '+' : '-'
  return `${sign}${value}`
}

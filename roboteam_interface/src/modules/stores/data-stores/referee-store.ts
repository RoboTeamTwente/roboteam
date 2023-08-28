import { defineStore } from 'pinia'
import { proto } from '../../../generated/proto'
import { readonly, shallowRef } from 'vue'
import ISSL_Referee = proto.ISSL_Referee

export const useRefereeDataStore = defineStore('refereeDataStore', () => {
  const data = shallowRef<ISSL_Referee[]>([])

  const pushRefereeData = (msg: ISSL_Referee[]) => {
      if (data.value.length + msg.length > 250) {
        console.warn('Too many referee messages, removing oldest')
        data.value = data.value.slice(msg.length)
      }

      data.value = [...data.value, ...msg]

      if (msg.length > 0) {
        console.log(data.value)
      }
  }

  return {
    data: readonly(data),
    pushRefereeData
  }
})

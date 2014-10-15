#define LEDCOMM_THREAD_STACK_SIZE 512

#ifdef __cplusplus
extern "C" {
#endif
    uint8_t ledCommClassify(uint16_t v);
    void ledCommPrepare(LEDCommDriver_t *ldp);
    void ledCommDetect(LEDCommDriver_t *ldp);
    uint8_t computeParity(uint8_t v);
    inline void ledCommOn(LEDCommDriver_t *ldp);
    inline void ledCommOff(LEDCommDriver_t *ldp);
    inline void ledCommInitPad(LEDCommDriver_t *ldp);
    void extcb1(EXTDriver *extp, uint8_t ch);
    msg_t LEDCommThread(void *arg);
    bool_t ledCommPollLinkStatus(LEDCommDriver_t *ldp);
#ifdef __cplusplus
}
#endif

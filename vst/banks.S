/* The three FSVR factory banks (../data, from FSVR's plugin/generated) embedded in the .so.
 * build.sh assembles this with -Wa,-I<data dir> so .incbin finds them. */
    .section .rodata
    .balign 4
    .global fsvr_voices_begin
    .hidden fsvr_voices_begin
fsvr_voices_begin:
    .incbin "fs1r_presets.syx"
    .global fsvr_voices_end
    .hidden fsvr_voices_end
fsvr_voices_end:
    .balign 4
    .global fsvr_perfs_begin
    .hidden fsvr_perfs_begin
fsvr_perfs_begin:
    .incbin "fs1r_performances.syx"
    .global fsvr_perfs_end
    .hidden fsvr_perfs_end
fsvr_perfs_end:
    .balign 4
    .global fsvr_fseqs_begin
    .hidden fsvr_fseqs_begin
fsvr_fseqs_begin:
    .incbin "fs1r_fseqs.syx"
    .global fsvr_fseqs_end
    .hidden fsvr_fseqs_end
fsvr_fseqs_end:
    .section .note.GNU-stack,"",%progbits

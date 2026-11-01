"""Exact pinned NativeLegacyDisplay transformation: guamp canvas (Display Zoom); default path unchanged.

iPhone layouts (Control Center grid, keyboard) assume a screen of at least 375 points; the 720 x 1600 panel at UI scale 2 is
360 x 800. With guamp-canvas-width/-height QuartzCore draws a larger canvas (750 x 1666 = 375 x 833 points) and the scanout copy
scales it to the panel, as the display pipe of an iPhone in Display Zoom (or an iPhone 6 Plus) does in hardware.
- guamp-canvas-mode=1 (default): canvas-width/canvas-height (4-byte words) on the display's provider. IOMobileFramebuffer's
  iomfb_get_display (0x1b63d1338..0x1b63d138c) reads them with _getEDTProperty (0x1b63d1b9c: IORegistryEntryGetParentEntry
  "IOService", IORegistryEntrySearchCFProperty) when no canvas_width/canvas_height preference exists;
  IOMobileFramebufferGetCanvasSizes (0x1b63d3b70) then lists the canvas after the panel size, and QuartzCore's
  CA::WindowServer::IOMFBDisplay::set_physical_and_canvas_sizes (0x1896df6fc) takes the last entry as the canvas
  (Display::set_size 0x1895fefc0: bounds from the canvas, physical size stays the panel).
- guamp-canvas-mode=2: the canvas is reported as the display dimensions (installBootDimensions).
- guamp-canvas-filter: 1 (default) bilinear, 0 nearest.
Swaps whose larger rectangle is the canvas are scaled into the GOP frame buffer (which the USB mirror keeps reading at 720 x 1600).
Without the two arguments the canvas is off. Section 11: the guamp display arguments (offline-legacy-*) and guamp-copy-gpu-wait=500
are code defaults since 28/10; an explicit key still wins.
"""
from pathlib import Path
import hashlib,sys
ORIGINAL_SHA256 = '09d7811db8fc6b659e12a05690867326e851f0d11bec7ffbb36b72c33ea035ca'
PATCHED_SHA256 = '56ffaee74bbb5276a5fbd2b01c181a01229611c33816525b6fe2c0a68affc918'
EDITS = []
# 1. Integer scaler, after the BGRA format test.
EDITS.append((r'''static inline bool nativeBgra(uint32_t format,uint32_t planes) {
    return (format==0x42475241 && !planes) || (format==0x26424741 && planes<=1);
}
''', r'''static inline bool nativeBgra(uint32_t format,uint32_t planes) {
    return (format==0x42475241 && !planes) || (format==0x26424741 && planes<=1);
}
// guamp canvas (scripts/guamp-canvas-variant.py): QuartzCore draws a canvas larger than the panel and the copy scales it down.
// Integer only: 8-bit weights (0..256), two 8-bit channels per 16-bit lane; premultiplied BGRA interpolates per channel.
extern "C" {volatile uint32_t guamp_canvas_state[4];volatile uint64_t guamp_canvas_frames,guamp_canvas_tables;}
static inline uint32_t guampLerp(uint32_t a,uint32_t b,uint32_t w) {
    const uint32_t v=256-w;
    const uint32_t rb=((((a&0x00ff00ffU)*v+(b&0x00ff00ffU)*w)+0x00800080U)>>8)&0x00ff00ffU;
    const uint32_t ga=((((a>>8)&0x00ff00ffU)*v+((b>>8)&0x00ff00ffU)*w)+0x00800080U)&0xff00ff00U;
    return rb|ga;
}
static inline uint64_t guampLerp2(uint64_t a,uint64_t b,uint64_t w) {
    const uint64_t mask=0x00ff00ff00ff00ffULL,round=0x0080008000800080ULL,v=256-w;
    const uint64_t rb=((((a&mask)*v+(b&mask)*w)+round)>>8)&mask;
    const uint64_t ga=((((a>>8)&mask)*v+((b>>8)&mask)*w)+round)&(mask<<8);
    return rb|ga;
}
// One output sample: input index << 9 | weight of the next input (0..256), centres aligned: (i + 1/2) * in / out - 1/2.
static inline uint32_t guampTap(uint32_t i,uint32_t outCount,uint32_t inCount,unsigned filter) {
    uint64_t position=(((uint64_t)(2*i+1)*inCount)<<16)/(2ull*outCount);
    position=position>=32768?position-32768:0;
    uint32_t index=(uint32_t)(position>>16),weight=(uint32_t)(((position&0xffff)+128)>>8);
    if(!filter){if(weight>=128)index=index+1;weight=0;}
    if(index>=inCount-1){index=inCount-1;weight=0;}
    return index<<9|weight;
}
struct GuampScaler {
    uint32_t sourceWidth,sourceHeight,width,height;
    uint32_t *line,*columns,*rows;   // one blended source row (sourceWidth + 2 words), then the column and the row taps
    vm_size_t size;
};
static inline uint32_t guampSample(const unsigned char *line,uint32_t tap) {
    const uint64_t at=(uint64_t)(tap>>9)*4;const uint32_t weight=tap&511;
    uint32_t a;__builtin_memcpy(&a,line+at,4);
    if(!weight)return a;
    uint32_t b;__builtin_memcpy(&b,line+at+4,4);
    return guampLerp(a,b,weight);
}
// Scales the visible source (sourceWidth x sourceHeight) into the width x height device-memory scanout: the two source rows of a
// weighted row tap are blended into the cached line, then two taps per output pixel, stored like native_legacy_copy_pixels.
static inline int guampScalePixels(volatile void *destination,uint64_t destinationBytes,uint64_t destinationStride,
    const void *source,uint64_t sourceBytes,uint64_t sourceStride,const GuampScaler &s) {
    const uintptr_t dst=(uintptr_t)destination,src=(uintptr_t)source;
    if (!s.line || (src&7) || (sourceStride&7) ||
        !native_legacy_copy_layout(destination,destinationBytes,destinationStride,source,sourceBytes,sourceStride,s.width,s.height) ||
        !native_legacy_surface_bounds(s.sourceWidth,s.sourceHeight,sourceStride,sourceBytes,sourceBytes)) return 0;
    const int wide=!(dst&7) && !(destinationStride&7);
    unsigned char *blended=reinterpret_cast<unsigned char *>(s.line);
    for (uint32_t y=0;y<s.height;++y) {
        const uint32_t row=s.rows[y]>>9,weight=s.rows[y]&511;
        const unsigned char *first=reinterpret_cast<const unsigned char *>(src+(uint64_t)row*sourceStride);
        const unsigned char *line=first;
        if (weight) {
            const unsigned char *second=first+sourceStride;
            for (uint32_t x=0;x+2<=s.sourceWidth;x+=2) {
                uint64_t a,b;__builtin_memcpy(&a,first+(uint64_t)x*4,8);__builtin_memcpy(&b,second+(uint64_t)x*4,8);
                const uint64_t value=guampLerp2(a,b,weight);__builtin_memcpy(blended+(uint64_t)x*4,&value,8);
            }
            if (s.sourceWidth&1) {
                const uint64_t at=(uint64_t)(s.sourceWidth-1)*4;uint32_t a,b;
                __builtin_memcpy(&a,first+at,4);__builtin_memcpy(&b,second+at,4);
                const uint32_t value=guampLerp(a,b,weight);__builtin_memcpy(blended+at,&value,4);
            }
            line=blended;
        }
        volatile uint32_t *out=(volatile uint32_t *)(dst+(uint64_t)y*destinationStride);
        uint32_t x=0;
        if (wide) {
            volatile uint64_t *out64=(volatile uint64_t *)out;
            for (;x+2<=s.width;x+=2) out64[x/2]=(uint64_t)guampSample(line,s.columns[x])|(uint64_t)guampSample(line,s.columns[x+1])<<32;
        }
        for (;x<s.width;++x) out[x]=guampSample(line,s.columns[x]);
    }
    return 1;
}
'''))
# 2. Canvas state, geometry and table helpers.
EDITS.append((r'''    bool gpuRetired=false;
''', r'''    bool gpuRetired=false;
    // guamp canvas: the actual scanout size, the canvas (0 when off), its mode and filter, and the scaler tables.
    uint32_t gopWidth=0,gopHeight=0,canvasWidth=0,canvasHeight=0;
    unsigned canvasMode=0,canvasFilter=1;
    GuampScaler scaler={};
    bool knownSize(uint32_t w,uint32_t h,uint32_t width,uint32_t height) const {
        return (w==width && h==height) || (canvasWidth && ((w==canvasWidth && h==canvasHeight) || (w==gopWidth && h==gopHeight)));
    }
    // Both swap rectangles at the origin, each the display, the canvas or the panel, and the request size likewise. The source is the
    // larger rectangle: the canvas when QuartzCore asks the display to scale it to the panel. Without a canvas only the display passes.
    bool swapGeometry(const unsigned char *bytes,uint32_t width,uint32_t height,uint32_t *sourceWidth,uint32_t *sourceHeight) const {
        uint32_t sw=0,sh=0;
        const unsigned rectOffsets[]={17,81};
        for(unsigned offset: rectOffsets) {
            uint32_t rect[4]={};memcpy(rect,bytes+offset,sizeof(rect));
            if(rect[0] || rect[1] || !knownSize(rect[2],rect[3],width,height))return false;
            if(rect[2]>sw)sw=rect[2];
            if(rect[3]>sh)sh=rect[3];
        }
        uint32_t size[2]={};memcpy(size,bytes+816,sizeof(size));
        if(!knownSize(size[0],size[1],width,height))return false;
        *sourceWidth=sw;*sourceHeight=sh;return sw && sh;
    }
    void freeScaler() {if(scaler.line)IOFree(scaler.line,scaler.size);scaler={};}
    bool scalerReady(uint32_t sourceWidth,uint32_t sourceHeight) {
        if(scaler.line && scaler.sourceWidth==sourceWidth && scaler.sourceHeight==sourceHeight && scaler.width==gopWidth &&
           scaler.height==gopHeight)return true;
        freeScaler();
        if(!gopWidth || !gopHeight || sourceWidth<gopWidth || sourceHeight<gopHeight || sourceWidth>=2*gopWidth ||
           sourceHeight>=2*gopHeight)return false;
        const vm_size_t size=((vm_size_t)sourceWidth+2+gopWidth+gopHeight)*4;
        uint32_t *table=static_cast<uint32_t *>(IOMalloc(size));
        if(!table)return false;
        uint32_t *columns=table+sourceWidth+2,*rows=columns+gopWidth;
        for(uint32_t x=0;x<gopWidth;++x)columns[x]=guampTap(x,gopWidth,sourceWidth,canvasFilter);
        for(uint32_t y=0;y<gopHeight;++y)rows[y]=guampTap(y,gopHeight,sourceHeight,canvasFilter);
        scaler={sourceWidth,sourceHeight,gopWidth,gopHeight,table,columns,rows,size};
        guamp_canvas_tables=guamp_canvas_tables+1;
        return true;
    }
    // The canvas arguments. Off unless the canvas is at least the panel and less than twice it each way (two taps cover at most a
    // 2:1 reduction); mode 1 also needs the provider to take both properties before any client opens the display.
    bool startCanvas(IOService *provider) {
        uint32_t w=750,h=1666;unsigned mode=1,filter=1;   // default 375x833 pt (owner 28/10); guamp-canvas-width=0 disables
        PE_parse_boot_argn("guamp-canvas-width",&w,sizeof(w));
        PE_parse_boot_argn("guamp-canvas-height",&h,sizeof(h));
        PE_parse_boot_argn("guamp-canvas-mode",&mode,sizeof(mode));
        PE_parse_boot_argn("guamp-canvas-filter",&filter,sizeof(filter));
        canvasWidth=0;canvasHeight=0;canvasMode=0;canvasFilter=filter?1:0;
        if(!w || !h)return false;
        if((mode!=1 && mode!=2) || w<gopWidth || h<gopHeight || w>=2*gopWidth || h>=2*gopHeight || (w==gopWidth && h==gopHeight)) {
            IOLog("GuampCanvas: %ux%u mode %u refused for the %ux%u panel\n",w,h,mode,gopWidth,gopHeight);return false;
        }
        if(mode==1) {
            OSData *width=OSData::withBytes(&w,sizeof(w)),*height=OSData::withBytes(&h,sizeof(h));
            const bool published=provider && width && height && provider->setProperty("canvas-width",width) &&
                                 provider->setProperty("canvas-height",height);
            OSSafeReleaseNULL(width);OSSafeReleaseNULL(height);
            if(!published){IOLog("GuampCanvas: canvas-width/canvas-height not published\n");return false;}
        }
        canvasWidth=w;canvasHeight=h;canvasMode=mode;
        guamp_canvas_state[0]=w;guamp_canvas_state[1]=h;guamp_canvas_state[2]=mode;guamp_canvas_state[3]=canvasFilter;
        IOLog("GuampCanvas: canvas %ux%u on the %ux%u panel, mode %u (%s), filter %u\n",w,h,gopWidth,gopHeight,mode,
              mode==1?"canvas sizes":"display dimensions",canvasFilter);
        return true;
    }
'''))
# 3. Release the tables with the object.
EDITS.append(('    void free() override {stopVsync();', '    void free() override {freeScaler();stopVsync();'))
# 4. Blank frames fill the actual scanout.
EDITS.append(('            uint32_t bw=0,bh=0;getDisplayDimensions(&bw,&bh);\n',
              '            uint32_t bw=0,bh=0;getDisplayDimensions(&bw,&bh);\n            if(canvasWidth){bw=gopWidth;bh=gopHeight;}\n'))
# 5. Copied frames: source = the swap's larger rectangle, destination = the actual scanout.
EDITS.append((r'''        uint32_t width=0,height=0;getDisplayDimensions(&width,&height);
        uint64_t length=(uint64_t)width*height*4;
        uint64_t sourceStride=native_legacy_surface_stride(surface);
        uint64_t allocation=native_legacy_surface_allocation(surface);
        if(!memory || !memory->metaCast("IOMemoryDescriptor") || !native_legacy_surface_bounds(width,height,sourceStride,allocation,memory->getLength()) ||
           native_legacy_surface_width(surface)<width || native_legacy_surface_height(surface)<height ||
''', r'''        uint32_t width=0,height=0;getDisplayDimensions(&width,&height);
        // guamp canvas: the source is the swap's larger rectangle, the destination the actual scanout.
        uint32_t sourceWidth=width,sourceHeight=height;bool geometry=true;
        if(canvasWidth) {
            geometry=swapGeometry(reinterpret_cast<const unsigned char *>(request),width,height,&sourceWidth,&sourceHeight);
            width=gopWidth;height=gopHeight;
        }
        const bool scaled=geometry && (sourceWidth!=width || sourceHeight!=height);
        uint64_t length=(uint64_t)width*height*4;
        uint64_t sourceStride=native_legacy_surface_stride(surface);
        uint64_t allocation=native_legacy_surface_allocation(surface);
        if(!geometry || (scaled && !scalerReady(sourceWidth,sourceHeight)) ||
           !memory || !memory->metaCast("IOMemoryDescriptor") || !native_legacy_surface_bounds(sourceWidth,sourceHeight,sourceStride,allocation,memory->getLength()) ||
           native_legacy_surface_width(surface)<sourceWidth || native_legacy_surface_height(surface)<sourceHeight ||
'''))
# 6. Latest-frame-wins (section 10) takes the frame here. Otherwise the GPU hand-off: a scaled frame is offered at its source size (the
#    presenter of AdrenoMetal 2ff4be9 scales it with ScaleKernelAir) unless guamp-gpu-scaled=0. The differential copy stays unscaled only.
EDITS.append(('        if(!gpuBypass && native_legacy_gpu_offer(this,memory,sourceStride,width,height)) {',
              r'''        // guamp latest-frame-wins (canvas variant section 10): the swap retires now, in order, and its frame becomes the newest one; the
        // GPU presenter or the copy thread takes it when the frame before it is done.
        if(latestOn()) {
            const GuampFrame frame={memory,sourceStride,allocation,length,video.v_rowBytes,width,height,
                                    scaled?sourceWidth:width,scaled?sourceHeight:height,scaled,false};
            memory->retain();latestRetire(request,frame);return;
        }
        if(!gpuBypass && (!scaled || gpuScaled) &&
           native_legacy_gpu_offer(this,memory,sourceStride,scaled?sourceWidth:width,scaled?sourceHeight:height)) {'''))
EDITS.append(('            if(!diffOff && shadow && shadowBytes>=need)', '            if(!diffOff && !scaled && shadow && shadowBytes>=need)'))
# 7. The scaled copy.
EDITS.append((r'''        if(stored==UINT64_MAX) {
            shadowValid=false;
''', r'''        if(stored==UINT64_MAX && scaled) {
            shadowValid=false;
            if(!guampScalePixels(reinterpret_cast<volatile void *>(bootMapping->getVirtualAddress()),bootMapping->getLength(),
                video.v_rowBytes,reinterpret_cast<const void *>(source->getVirtualAddress()),source->getLength(),sourceStride,scaler)) {
                OSSafeReleaseNULL(source);native_legacy_transfer_observed(kIOReturnBadArgument,request,0);return;
            }
            guamp_canvas_frames=guamp_canvas_frames+1;stored=length;
        }
        if(stored==UINT64_MAX) {
            shadowValid=false;
'''))
# 8. Validation: canvas-sized rectangles and request size, the surface checked against the source.
EDITS.append((r'''                reason=3;bool rectangles=true;
                const unsigned rectOffsets[]={17,81};
                for(unsigned offset: rectOffsets) {
                    uint32_t rect[4]={};memcpy(rect,bytes+offset,sizeof(rect));
                    rectangles=rectangles && rect[0]==0 && rect[1]==0 && rect[2]==width && rect[3]==height;
                }
''', r'''                reason=3;uint32_t sourceWidth=0,sourceHeight=0;
                bool rectangles=swapGeometry(bytes,width,height,&sourceWidth,&sourceHeight);
'''))
EDITS.append(('if(width && height && rectangles && options && word(816)==width && word(820)==height) {',
              'if(width && height && rectangles && options) {'))
EDITS.append(('if(native_legacy_surface_width(surface)>=width && native_legacy_surface_height(surface)>=height &&',
              'if(native_legacy_surface_width(surface)>=sourceWidth && native_legacy_surface_height(surface)>=sourceHeight &&'))
EDITS.append(('native_legacy_surface_bounds(width,height,native_legacy_surface_stride(surface),',
              'native_legacy_surface_bounds(sourceWidth,sourceHeight,native_legacy_surface_stride(surface),'))
EDITS.append(('word(816)==width && word(820)==height && native_legacy_blank_state(',
              'knownSize(word(816),word(820),width,height) && native_legacy_blank_state('))
# 9. Start: the actual GOP size stays the scanout; mode 2 reports the canvas as the display dimensions.
EDITS.append((r'''            installBootDimensions((uint32_t)video.v_width,(uint32_t)video.v_height);
            uint32_t width=0,height=0;getDisplayDimensions(&width,&height);
            valid=width==video.v_width && height==video.v_height;
            if(valid) {
                OSSafeReleaseNULL(bootMapping);bootMapping=mapping;mapping=nullptr;primary=video.v_display==1;
                valid=native_legacy_gop_mapping_observed(this,bootMapping,width,height);
''', r'''            gopWidth=(uint32_t)video.v_width;gopHeight=(uint32_t)video.v_height;
            const bool canvas=startCanvas(provider);
            const uint32_t reportedWidth=canvas && canvasMode==2?canvasWidth:gopWidth;
            const uint32_t reportedHeight=canvas && canvasMode==2?canvasHeight:gopHeight;
            installBootDimensions(reportedWidth,reportedHeight);
            uint32_t width=0,height=0;getDisplayDimensions(&width,&height);
            valid=width==reportedWidth && height==reportedHeight;
            if(valid) {
                OSSafeReleaseNULL(bootMapping);bootMapping=mapping;mapping=nullptr;primary=video.v_display==1;
                valid=native_legacy_gop_mapping_observed(this,bootMapping,gopWidth,gopHeight);
'''))
# 9. Faster CPU copy of the swapped surface, each part behind its own boot-arg (all default off):
#    guamp-fast-copy=1        the surface is read through a cacheable kernel mapping (kIOMapCopybackCache), cleaned and invalidated
#                             by VA first so lines another CPU left dirty reach memory and no stale line is read;
#    guamp-fast-copy-flush=0  skips that cache maintenance (only right if the render server's own mapping is cacheable too);
#    guamp-map-cache=1        keeps the kernel mappings of the last 4 surfaces (per memory descriptor, oldest evicted) instead of
#                             mapping every frame; the descriptor stays retained while cached;
#    guamp-async-copy=1       the swap retires at once and a kernel thread call copies it, as the GPU offer path does (early retire,
#                             transferred held until the copy ends, gpuComplete on the work loop); a failed copy falls back to the
#                             synchronous copy for the next frame (gpuBypass);
#    guamp-async-copy=2       snapshot: the surface is first copied (cacheable, fast) into an internal buffer, the swap retires and
#                             the thread call scales from the buffer, so a surface the render server reuses is never read late;
#    guamp-copy-probe=N       the first N synchronous frames also time a read-only pass over the surface.
#    guamp_copy_split: [0] frames mapped, [1] map + cache maintenance ticks, [2] probed frames, [3] probe ticks, [4] mapping cache
#    mode, [5] last mapping cacheable, [6] probe bytes, [7] cacheable mappings refused, [8] asynchronous copies, [9] cached mappings
#    reused, [10] async skipped: option off, [11] skipped: GPU bypass after a failure, [12] skipped: no thread call, [13] skipped:
#    a copy still in flight, [14] options read (bit0 fast, bit1 flush, bit2 map cache, bits3-4 async mode, bit5 read, bit6 GPU wait), [15] snapshot
#    ticks (copy into the internal buffer).
#    guamp-copy-gpu-wait=<ms> (default 500 since 28/10, 0 off; at most 1000): the copy is deferred to the thread call (guamp-async-copy is taken as
#                             1, no snapshot) and first waits, at most <ms>, until the GPU has retired every submission handed out
#                             by then (drivers/GuampAdreno guamp_adreno_wait_submitted: the CP timestamp of the fence page against
#                             the ring's newest fence), because the render server swaps after waitUntilScheduled, before its Metal
#                             frame is written. The swap still retires at once; the surface stays held until the next swap, which
#                             is not transferred before this copy ends. Then it maps (and cleans/invalidates) and copies as before.
#    guamp_copy_gpu_wait: [0] waits that saw the fence reached, [1] timed out (copied anyway), [2] no GPU ring (copied at once),
#    [3] microseconds waited in total ([0] and [1]), [4] longest wait (us), [5] fence already reached (under 20 us), [6] the bound
#    in milliseconds.
EDITS.append(('extern "C" {volatile uint32_t guamp_canvas_state[4];volatile uint64_t guamp_canvas_frames,guamp_canvas_tables;}',
              'extern "C" {volatile uint32_t guamp_canvas_state[4];volatile uint64_t guamp_canvas_frames,guamp_canvas_tables;}\n'
              '#include <kern/thread_call.h>\n'
              'extern "C" {volatile uint64_t guamp_copy_split[16];volatile uint64_t guamp_copy_probe_sink;}\n'
              'extern "C" void flush_dcache64(addr64_t,unsigned,int);\n'
              '// guamp-copy-gpu-wait: drivers/GuampAdreno defines it; a kernel without the GPU probe has no ring (static link: weak definition).\n'
              'extern "C" {volatile uint64_t guamp_copy_gpu_wait[8];\n'
              '__attribute__((weak)) int guamp_adreno_wait_submitted(uint32_t,uint32_t *waited){if(waited)*waited=0;return -1;}\n'
              '__attribute__((weak)) int guamp_adreno_ring_marks(uint32_t *submitted,uint32_t *retired)\n'
              '    {if(submitted)*submitted=0;if(retired)*retired=0;return -1;}\n'
              '// Section 10 counters: [0] frames started, [1] coalesced (replaced before they started), [2] stale-skipped (a newer frame was\n'
              '// queued when the GPU wait ended), [3] GPU frames, [4] CPU frames, [5] in-thread refreshes, [6] trailing re-shows, [7] GPU\n'
              '// failures, [8] CPU copy failures, [9] latest-frame-wins on.\n'
              'volatile uint64_t guamp_copy_latest[10];}'))
EDITS.append(('    GuampScaler scaler={};\n', r'''    GuampScaler scaler={};
    // guamp copy options (scripts/guamp-canvas-variant.py section 9), read once.
    int copyOptions=-1;bool fastCopy=false,fastFlush=true,mapCache=false;unsigned asyncMode=0,probeBudget=0,gpuWaitMs=500;   // guamp-copy-gpu-wait default 500 (verified 28/10)
    bool latestWanted=false,gpuScaled=true;unsigned trailMs=0;
    // guamp-copy-min-ms (31/10): the shortest time between two CPU copies into the scanout. The panel is broken and only the USB
    // mirror (about 2 frames/s) shows the screen, while backboardd swapped ~24 frames/s of 4.5 MB each (25-29 ms of copy, ~half a
    // core of kernel_task). Frames swapped in between coalesce (latest-frame-wins), the last one is always shown. 0 = no limit.
    unsigned copyMinMs=100;uint64_t lastCpuStart=0;
    void readCopyOptions() {
        if(copyOptions>=0)return;
        unsigned v=1;PE_parse_boot_argn("guamp-fast-copy",&v,sizeof v);fastCopy=v==1;   // default on (owner 28/10)
        v=1;PE_parse_boot_argn("guamp-fast-copy-flush",&v,sizeof v);fastFlush=v!=0;
        v=1;PE_parse_boot_argn("guamp-map-cache",&v,sizeof v);mapCache=v==1;   // default on (owner 28/10)
        v=0;PE_parse_boot_argn("guamp-async-copy",&v,sizeof v);asyncMode=v<=2?v:0;
        PE_parse_boot_argn("guamp-copy-probe",&probeBudget,sizeof probeBudget);if(probeBudget>64)probeBudget=64;
        // guamp-copy-gpu-wait: the GPU wait runs in the deferred copy, after the swap retired; a snapshot would read too early.
        PE_parse_boot_argn("guamp-copy-gpu-wait",&gpuWaitMs,sizeof gpuWaitMs);if(gpuWaitMs>1000)gpuWaitMs=1000;
        if(gpuWaitMs)asyncMode=1;
        guamp_copy_gpu_wait[6]=gpuWaitMs;
        // Section 10: latest-frame-wins with the deferred copy (asyncMode 1) or the GPU hand-off (sheng-fb-gpu=1); the trailing check
        // and the GPU's own scaling of canvas frames.
        v=1;PE_parse_boot_argn("guamp-swap-latest",&v,sizeof v);
        unsigned handoff=0;PE_parse_boot_argn("sheng-fb-gpu",&handoff,sizeof handoff);
        latestWanted=v==1 && (asyncMode==1 || handoff==1);
        trailMs=25;PE_parse_boot_argn("guamp-copy-trail",&trailMs,sizeof trailMs);if(trailMs>200)trailMs=200;
        v=1;PE_parse_boot_argn("guamp-gpu-scaled",&v,sizeof v);gpuScaled=v!=0;
        PE_parse_boot_argn("guamp-copy-min-ms",&copyMinMs,sizeof copyMinMs);if(copyMinMs>1000)copyMinMs=1000;
        guamp_copy_latest[9]=latestWanted?1:0;
        copyOptions=1;
        guamp_copy_split[14]=(fastCopy?1:0)|(fastFlush?2:0)|(mapCache?4:0)|((uint64_t)asyncMode<<3)|32|(gpuWaitMs?64:0)|
                             (latestWanted?128:0)|(gpuScaled?256:0);
    }
    struct GuampMapSlot {IOMemoryDescriptor *memory;IOMemoryMap *map;uint64_t used;bool cacheable;};
    GuampMapSlot mapSlots[4]={};uint64_t mapUse=0;
    void freeMapSlots() {for(auto &s: mapSlots){OSSafeReleaseNULL(s.map);OSSafeReleaseNULL(s.memory);s.used=0;s.cacheable=false;}}
    // A read-only kernel mapping of the surface that the caller releases (a cached one carries an extra reference).
    IOMemoryMap *guampSourceMap(IOMemoryDescriptor *memory,uint64_t allocation) {
        readCopyOptions();
        uint64_t mapStart;__asm__ volatile("isb\n\tmrs %0, cntpct_el0":"=r"(mapStart));
        IOMemoryMap *source=nullptr;bool cacheable=false;
        if(mapCache)
            for(auto &s: mapSlots)
                if(s.memory==memory && s.map) {
                    source=s.map;source->retain();s.used=++mapUse;cacheable=s.cacheable;guamp_copy_split[9]=guamp_copy_split[9]+1;break;
                }
        if(!source) {
            if(fastCopy) {
                source=memory->map(kIOMapAnywhere|kIOMapReadOnly|kIOMapCopybackCache);
                if(!source || !source->getVirtualAddress() || source->getLength()<allocation || source->getLength()>0xffffffffull)
                    {OSSafeReleaseNULL(source);guamp_copy_split[7]=guamp_copy_split[7]+1;}
                else cacheable=true;
            }
            if(!source)source=memory->map(kIOMapAnywhere|kIOMapReadOnly);
            if(source && mapCache) {
                GuampMapSlot *slot=&mapSlots[0];
                for(auto &s: mapSlots) {if(!s.map){slot=&s;break;} if(s.used<slot->used)slot=&s;}
                OSSafeReleaseNULL(slot->map);OSSafeReleaseNULL(slot->memory);
                memory->retain();source->retain();slot->memory=memory;slot->map=source;slot->used=++mapUse;slot->cacheable=cacheable;
            }
        }
        if(source && cacheable && fastFlush && source->getVirtualAddress() && source->getLength()<=0xffffffffull)
            flush_dcache64((addr64_t)source->getVirtualAddress(),(unsigned)source->getLength(),0);
        if(source) {
            uint64_t mapEnd;__asm__ volatile("isb\n\tmrs %0, cntpct_el0":"=r"(mapEnd));
            guamp_copy_split[0]=guamp_copy_split[0]+1;guamp_copy_split[1]=guamp_copy_split[1]+(mapEnd-mapStart);
            guamp_copy_split[4]=(source->getMapOptions()&kIOMapCacheMask)>>kIOMapCacheShift;guamp_copy_split[5]=cacheable?1:0;
        }
        return source;
    }
    // guamp-async-copy=1: one copy in flight; its parameters are taken under the work loop before the thread call runs.
    thread_call_t asyncCall=nullptr;IOMemoryDescriptor *asyncMemory=nullptr;
    uint64_t asyncStride=0,asyncAllocation=0,asyncLength=0,asyncRowBytes=0;uint32_t asyncWidth=0,asyncHeight=0;bool asyncScaled=false;
    bool asyncBusy=false,asyncSnapshot=false;void *snapshot=nullptr;vm_size_t snapshotSize=0;uint64_t snapshotBytes=0;
    static void asyncCopyEntry(thread_call_param_t owner,thread_call_param_t) {static_cast<NativeLegacyDisplay *>(owner)->asyncCopy();}
    // guamp-copy-gpu-wait: at most gpuWaitMs until the GPU retired everything handed out by now (thread context only).
    void guampGpuWait() {
        if(!gpuWaitMs)return;
        uint32_t waited=0;const int r=guamp_adreno_wait_submitted(gpuWaitMs*1000,&waited);
        if(r<0)guamp_copy_gpu_wait[2]=guamp_copy_gpu_wait[2]+1;
        else {
            if(r)guamp_copy_gpu_wait[1]=guamp_copy_gpu_wait[1]+1;else guamp_copy_gpu_wait[0]=guamp_copy_gpu_wait[0]+1;
            if(!r && waited<20)guamp_copy_gpu_wait[5]=guamp_copy_gpu_wait[5]+1;
            guamp_copy_gpu_wait[3]=guamp_copy_gpu_wait[3]+waited;
            if(waited>guamp_copy_gpu_wait[4])guamp_copy_gpu_wait[4]=waited;
        }
    }
    void asyncCopy() {
        IOMemoryDescriptor *memory=asyncMemory;asyncMemory=nullptr;
        int status=-1;
        if(memory)guampGpuWait();
        IOMemoryMap *source=memory?guampSourceMap(memory,asyncAllocation):nullptr;
        const uintptr_t from=asyncSnapshot?(uintptr_t)snapshot:(source?(uintptr_t)source->getVirtualAddress():0);
        const uint64_t fromBytes=asyncSnapshot?snapshotBytes:(source?source->getLength():0);
        if(from && fromBytes>=asyncAllocation && bootMapping) {
            uint64_t copyStart;__asm__ volatile("isb\n\tmrs %0, cntpct_el0":"=r"(copyStart));
            const int ok=asyncScaled
                ?guampScalePixels(reinterpret_cast<volatile void *>(bootMapping->getVirtualAddress()),bootMapping->getLength(),asyncRowBytes,
                    reinterpret_cast<const void *>(from),fromBytes,asyncStride,scaler)
                :native_legacy_copy_pixels(reinterpret_cast<volatile void *>(bootMapping->getVirtualAddress()),bootMapping->getLength(),
                    asyncRowBytes,reinterpret_cast<const void *>(from),fromBytes,asyncStride,asyncWidth,asyncHeight);
            __asm__ volatile("dsb sy" ::: "memory");
            uint64_t copyEnd;__asm__ volatile("isb\n\tmrs %0, cntpct_el0":"=r"(copyEnd));
            if(ok){native_legacy_copy_timing_observed(copyEnd-copyStart,asyncLength);if(asyncScaled)guamp_canvas_frames=guamp_canvas_frames+1;status=0;}
        }
        OSSafeReleaseNULL(source);OSSafeReleaseNULL(memory);asyncBusy=false;
        gpuComplete(status);
    }
    void freeAsync() {
        if(asyncCall){thread_call_cancel_wait(asyncCall);thread_call_free(asyncCall);asyncCall=nullptr;}
        OSSafeReleaseNULL(asyncMemory);freeMapSlots();
        if(snapshot){IOFreeAligned(snapshot,snapshotSize);snapshot=nullptr;snapshotSize=0;}
    }
'''))
EDITS.append(('    void free() override {freeScaler();stopVsync();', '    void free() override {freeAsync();freeScaler();stopVsync();'))
EDITS.append(('        IOMemoryMap *source=memory->map(kIOMapAnywhere|kIOMapReadOnly);\n',
r'''        readCopyOptions();
        if(!asyncMode)guamp_copy_split[10]=guamp_copy_split[10]+1;
        else if(gpuBypass)guamp_copy_split[11]=guamp_copy_split[11]+1;
        else {
            if(!asyncCall)asyncCall=thread_call_allocate_with_options(asyncCopyEntry,this,THREAD_CALL_PRIORITY_KERNEL_HIGH,0);
            if(!asyncCall)guamp_copy_split[12]=guamp_copy_split[12]+1;
            else if(asyncBusy || asyncMemory)guamp_copy_split[13]=guamp_copy_split[13]+1;
            else {
                bool ready=true;asyncSnapshot=false;
                if(asyncMode==2) {
                    // Snapshot: the surface into the internal buffer now, so retiring the swap cannot race the render server.
                    if(!snapshot || snapshotSize<allocation) {
                        if(snapshot)IOFreeAligned(snapshot,snapshotSize);
                        snapshotSize=(vm_size_t)((allocation+16383)&~16383ull);snapshot=IOMallocAligned(snapshotSize,16384);
                        if(!snapshot)snapshotSize=0;
                    }
                    IOMemoryMap *copyFrom=snapshot?guampSourceMap(memory,allocation):nullptr;
                    ready=copyFrom && copyFrom->getVirtualAddress() && copyFrom->getLength()>=allocation;
                    if(ready) {
                        uint64_t snapStart,snapEnd;__asm__ volatile("isb\n\tmrs %0, cntpct_el0":"=r"(snapStart));
                        memcpy(snapshot,reinterpret_cast<const void *>(copyFrom->getVirtualAddress()),(size_t)allocation);
                        __asm__ volatile("isb\n\tmrs %0, cntpct_el0":"=r"(snapEnd));
                        guamp_copy_split[15]=guamp_copy_split[15]+(snapEnd-snapStart);
                        snapshotBytes=allocation;asyncSnapshot=true;
                    }
                    OSSafeReleaseNULL(copyFrom);
                }
                if(ready) {
                    if(!asyncSnapshot){memory->retain();asyncMemory=memory;}
                    asyncStride=sourceStride;asyncAllocation=allocation;asyncLength=length;
                    asyncRowBytes=video.v_rowBytes;asyncWidth=width;asyncHeight=height;asyncScaled=scaled;asyncBusy=true;
                    native_legacy_swap_stage(1);
                    shadowValid=false;gpuRequest=request;gpuLength=length;transferred=true;
                    if(retireSwap(request,true)){gpuRetired=true;native_legacy_early_retires=native_legacy_early_retires+1;}
                    guamp_copy_split[8]=guamp_copy_split[8]+1;
                    thread_call_enter(asyncCall);
                    return;
                }
            }
        }
        IOMemoryMap *source=guampSourceMap(memory,allocation);
        if(source && probeBudget && source->getVirtualAddress()) {
            --probeBudget;
            const uint64_t *word=reinterpret_cast<const uint64_t *>(source->getVirtualAddress());
            const uint64_t words=((source->getLength()<allocation?source->getLength():allocation)/32)*4;
            uint64_t sum=0,probeStart,probeEnd;__asm__ volatile("isb\n\tmrs %0, cntpct_el0":"=r"(probeStart));
            for(uint64_t i=0;i<words;i+=4)sum+=word[i]^word[i+1]^word[i+2]^word[i+3];
            __asm__ volatile("isb\n\tmrs %0, cntpct_el0":"=r"(probeEnd));
            guamp_copy_probe_sink=sum;guamp_copy_split[2]=guamp_copy_split[2]+1;
            guamp_copy_split[3]=guamp_copy_split[3]+(probeEnd-probeStart);guamp_copy_split[6]=words*8;
        }
'''))
# 10. Latest-frame-wins (guamp-swap-latest, default 1; on with the deferred copy, i.e. guamp-async-copy=1 or guamp-copy-gpu-wait, or with
#     the GPU hand-off sheng-fb-gpu=1). Every validated swap retires at once, in queue order (the render server never waits behind a copy
#     or a GPU wait), and its frame becomes the newest: a frame that has not started yet is replaced (coalesced). One frame is in flight at
#     a time, with the GPU presenter when it takes the offer, else with the copy thread. The copy thread waits for the GPU (guamp-copy-gpu-
#     wait), drops the frame when a newer one was queued meanwhile (stale-skipped), copies it, and while the GPU took more work since its
#     wait (the frame's own rendering may have reached the ring after the swap) waits and copies it again, at most twice. When nothing is
#     queued, the frame on screen is checked again every guamp-copy-trail=<ms> (default 25, 0 off, at most 4 times after a swap): if the
#     GPU ran anything since it was copied, it is shown again. A blank swap is a frame too (filled by the thread). A frame the GPU loses
#     goes to the copy thread. guamp-swap-latest=0 keeps the previous one-frame-in-flight latch.
EDITS.append(('struct GuampScaler {', r'''// guamp latest-frame-wins (scripts/guamp-canvas-variant.py section 10). The text between the two markers is the host-tested core
// (drivers/NativeEmmc/tests/latest_host.cpp extracts it). Runs on the display's work loop, except superseded() (any thread).
// Env: latestOffer(frame) (true if the GPU took it), latestStartCpu() (the copy thread takes cpuFrame), latestRetain/latestRelease.
// guamp-latest-core-begin
template<class Frame,class Env> struct GuampLatest {
    Frame next{},cpuFrame{},gpuFrame{},shown{};
    bool queued=false,cpu=false,gpu=false,hasShown=false,cpuOnce=false;
    uint64_t started=0,coalesced=0,stale=0,gpuFrames=0,cpuFrames=0,gpuFailures=0,cpuFailures=0,trails=0;
    void setQueued(bool value) {__atomic_store_n(&queued,value,__ATOMIC_RELEASE);}
    // A retired swap's frame (holding one reference, handed over) becomes the newest; a queued frame that never started is dropped.
    void swapped(Env &env,const Frame &frame) {
        if(queued){env.latestRelease(next);coalesced=coalesced+1;}
        next=frame;setQueued(true);dispatch(env);
    }
    // Starts the newest frame when none is in flight: the GPU presenter if it takes it, else the copy thread.
    void dispatch(Env &env) {
        if(!queued || cpu || gpu)return;
        const Frame frame=next;next=Frame{};setQueued(false);started=started+1;
        if(!cpuOnce && env.latestOffer(frame)){gpuFrame=frame;gpu=true;return;}
        cpuOnce=false;cpuFrame=frame;cpu=true;env.latestStartCpu();
    }
    // The copy thread, after its GPU wait: a newer frame is queued, so this one need not be copied.
    bool superseded() const {return __atomic_load_n(&queued,__ATOMIC_ACQUIRE);}
    // The copy thread is done: 0 copied, 1 dropped as superseded, 2 the copy failed.
    void cpuDone(Env &env,int result) {
        const Frame frame=cpuFrame;cpuFrame=Frame{};cpu=false;
        if(!result){cpuFrames=cpuFrames+1;keep(env,frame);}
        else {if(result==1)stale=stale+1;else cpuFailures=cpuFailures+1;env.latestRelease(frame);}
        dispatch(env);
    }
    // The GPU presenter is done (0) or lost the frame: a lost frame goes to the copy thread unless a newer one is queued.
    void gpuDone(Env &env,int status) {
        const Frame frame=gpuFrame;gpuFrame=Frame{};gpu=false;
        if(!status){gpuFrames=gpuFrames+1;keep(env,frame);}
        else {
            gpuFailures=gpuFailures+1;cpuOnce=true;
            if(queued)env.latestRelease(frame);else{next=frame;setQueued(true);}
        }
        dispatch(env);
    }
    // Shows the frame on screen again (its surface may have changed since it was copied).
    bool trail(Env &env) {
        if(queued || cpu || gpu || !hasShown)return false;
        next=shown;env.latestRetain(next);setQueued(true);trails=trails+1;dispatch(env);return true;
    }
    void keep(Env &env,const Frame &frame) {if(hasShown)env.latestRelease(shown);shown=frame;hasShown=true;}
    bool idle() const {return !queued && !cpu && !gpu;}
    // Teardown, with the copy thread stopped: every reference held is dropped.
    void clear(Env &env) {
        if(queued)env.latestRelease(next);
        if(cpu)env.latestRelease(cpuFrame);
        if(gpu)env.latestRelease(gpuFrame);
        if(hasShown)env.latestRelease(shown);
        next=cpuFrame=gpuFrame=shown=Frame{};setQueued(false);cpu=gpu=hasShown=cpuOnce=false;
    }
};
// guamp-latest-core-end
// One frame of section 10: the surface (retained; null for a blank swap) and how it reaches the scanout.
struct GuampFrame {
    IOMemoryDescriptor *memory;uint64_t stride,allocation,length,rowBytes;
    uint32_t width,height,offerWidth,offerHeight;bool scaled,blank;
};
struct GuampScaler {'''))
EDITS.append(('    void freeAsync() {\n', r'''    // Section 10: latest-frame-wins.
    template<class,class> friend struct GuampLatest;
    GuampLatest<GuampFrame,NativeLegacyDisplay> latest;
    thread_call_t latestCall=nullptr,trailCall=nullptr;
    uint32_t shownMark=0;unsigned trailLeft=0;
    static void latestCopyEntry(thread_call_param_t owner,thread_call_param_t) {static_cast<NativeLegacyDisplay *>(owner)->latestCopy();}
    static void trailEntry(thread_call_param_t owner,thread_call_param_t) {
        NativeLegacyDisplay *display=static_cast<NativeLegacyDisplay *>(owner);IOWorkLoop *loop=display->getWorkLoop();
        auto action=[](OSObject *target,void *,void *,void *,void *)->IOReturn {
            static_cast<NativeLegacyDisplay *>(target)->trailTick();return kIOReturnSuccess;
        };
        if(loop)loop->runAction(action,display);
    }
    bool latestOn() {
        readCopyOptions();
        if(!latestWanted)return false;
        if(!latestCall)latestCall=thread_call_allocate_with_options(latestCopyEntry,this,THREAD_CALL_PRIORITY_KERNEL_HIGH,0);
        if(!trailCall && trailMs)trailCall=thread_call_allocate_with_options(trailEntry,this,THREAD_CALL_PRIORITY_KERNEL_HIGH,0);
        return latestCall!=nullptr;
    }
    void latestRetain(const GuampFrame &frame) {if(frame.memory)frame.memory->retain();}
    void latestRelease(const GuampFrame &frame) {if(frame.memory)frame.memory->release();}
    bool latestOffer(const GuampFrame &frame) {
        return frame.memory && (!frame.scaled || gpuScaled) &&
               native_legacy_gpu_offer(this,frame.memory,frame.stride,frame.offerWidth,frame.offerHeight);
    }
    void latestStartCpu() {
        guamp_copy_split[8]=guamp_copy_split[8]+1;
        const uint64_t now=mach_absolute_time();uint64_t gap=0;
        if(copyMinMs)nanoseconds_to_absolutetime((uint64_t)copyMinMs*1000000ULL,&gap);
        if(gap && lastCpuStart && now-lastCpuStart<gap) {   // too soon: start when the interval is over, newer frames coalesce meanwhile
            lastCpuStart=lastCpuStart+gap;thread_call_enter_delayed(latestCall,lastCpuStart);
        } else {
            lastCpuStart=now;thread_call_enter(latestCall);
        }
    }
    void latestPublish() {
        guamp_copy_latest[0]=latest.started;guamp_copy_latest[1]=latest.coalesced;guamp_copy_latest[2]=latest.stale;
        guamp_copy_latest[3]=latest.gpuFrames;guamp_copy_latest[4]=latest.cpuFrames;guamp_copy_latest[6]=latest.trails;
        guamp_copy_latest[7]=latest.gpuFailures;guamp_copy_latest[8]=latest.cpuFailures;
    }
    // The validated head: retired now (in queue order), its frame handed to the latest-frame machine.
    void latestRetire(const OriginalLegacySwapRequest *request,const GuampFrame &frame) {
        native_legacy_swap_stage(1);shadowValid=false;trailLeft=4;
        latest.swapped(*this,frame);latestPublish();
        transferred=true;native_legacy_transfer_observed(kIOReturnSuccess,request,frame.length);
        finishTransfer(request,true);
    }
    // Nothing queued or in flight: look at the frame on screen again later (trailTick).
    void latestIdle() {
        if(!latest.idle() || !latest.hasShown || !trailCall || !trailMs || !trailLeft)return;
        uint64_t deadline=0;clock_interval_to_deadline(trailMs,kMillisecondScale,&deadline);
        thread_call_enter_delayed(trailCall,deadline);
    }
    void trailTick() {
        if(!trailLeft || !latest.idle() || !latest.hasShown)return;
        trailLeft=trailLeft-1;
        uint32_t now=0;
        if(guamp_adreno_ring_marks(&now,nullptr))return;   // no GPU ring: nothing can have changed the surface behind the swap
        if(now!=shownMark)latest.trail(*this);else latestIdle();
        latestPublish();
    }
    void latestCpuDone(int result) {latest.cpuDone(*this,result);latestPublish();latestIdle();}
    void latestGpuDone(int status) {
        uint32_t now=0;
        if(!status && !guamp_adreno_ring_marks(&now,nullptr))shownMark=now;
        latest.gpuDone(*this,status);latestPublish();latestIdle();
    }
    // One copy of the frame into the scanout (copy thread).
    bool latestCopyOnce(const GuampFrame &frame) {
        IOMemoryMap *source=guampSourceMap(frame.memory,frame.allocation);
        bool ok=false;
        if(source && source->getVirtualAddress() && source->getLength()>=frame.allocation && bootMapping) {
            uint64_t copyStart;__asm__ volatile("isb\n\tmrs %0, cntpct_el0":"=r"(copyStart));
            ok=(frame.scaled
                ?guampScalePixels(reinterpret_cast<volatile void *>(bootMapping->getVirtualAddress()),bootMapping->getLength(),frame.rowBytes,
                    reinterpret_cast<const void *>(source->getVirtualAddress()),source->getLength(),frame.stride,scaler)
                :native_legacy_copy_pixels(reinterpret_cast<volatile void *>(bootMapping->getVirtualAddress()),bootMapping->getLength(),
                    frame.rowBytes,reinterpret_cast<const void *>(source->getVirtualAddress()),source->getLength(),frame.stride,
                    frame.width,frame.height))!=0;
            __asm__ volatile("dsb sy" ::: "memory");
            uint64_t copyEnd;__asm__ volatile("isb\n\tmrs %0, cntpct_el0":"=r"(copyEnd));
            if(ok){native_legacy_copy_timing_observed(copyEnd-copyStart,frame.length);if(frame.scaled)guamp_canvas_frames=guamp_canvas_frames+1;}
        }
        OSSafeReleaseNULL(source);return ok;
    }
    // The copy thread: one frame (latest.cpuFrame, set before the call was entered), then back to the work loop.
    void latestCopy() {
        const GuampFrame frame=latest.cpuFrame;
        int result=2;uint32_t mark=0;
        const bool marked=guamp_adreno_ring_marks(&mark,nullptr)==0;
        if(frame.blank) {
            if(bootMapping && native_legacy_fill_pixels(reinterpret_cast<volatile void *>(bootMapping->getVirtualAddress()),
                bootMapping->getLength(),frame.rowBytes,frame.width,frame.height,0xff000000U)) {
                __asm__ volatile("dsb sy" ::: "memory");result=0;
            }
        } else if(frame.memory) {
            guampGpuWait();
            if(latest.superseded())result=1;
            else if(latestCopyOnce(frame)) {
                result=0;
                // The GPU took more work since the wait: perhaps this frame's own rendering, submitted after its swap. Wait for it and
                // copy again, at most twice, unless a newer frame is queued.
                for(unsigned again=0;marked && gpuWaitMs && again<2 && !latest.superseded();++again) {
                    uint32_t now=0;
                    if(guamp_adreno_ring_marks(&now,nullptr) || now==mark)break;
                    mark=now;guampGpuWait();
                    if(latest.superseded() || !latestCopyOnce(frame))break;
                    guamp_copy_latest[5]=guamp_copy_latest[5]+1;
                }
            }
        }
        shownMark=mark;
        IOWorkLoop *loop=getWorkLoop();
        auto action=[](OSObject *target,void *value,void *,void *,void *)->IOReturn {
            static_cast<NativeLegacyDisplay *>(target)->latestCpuDone((int)(uintptr_t)value);return kIOReturnSuccess;
        };
        if(loop)loop->runAction(action,this,reinterpret_cast<void *>((uintptr_t)result));else latestCpuDone(result);
    }
    void freeAsync() {
        if(latestCall){thread_call_cancel_wait(latestCall);thread_call_free(latestCall);latestCall=nullptr;}
        if(trailCall){thread_call_cancel_wait(trailCall);thread_call_free(trailCall);trailCall=nullptr;}
        latest.clear(*this);
'''))
# The GPU presenter's completion of a section-10 frame.
EDITS.append(('''    void finishGpuSwap(int status) {
        const OriginalLegacySwapRequest *request=gpuRequest;
''', '''    void finishGpuSwap(int status) {
        if(latest.gpu){latestGpuDone(status);return;}   // guamp latest-frame-wins (canvas variant section 10)
        const OriginalLegacySwapRequest *request=gpuRequest;
'''))
# A blank swap is a section-10 frame as well, so it cannot be overtaken by an older frame's copy.
EDITS.append(('''            if(!native_legacy_prepare_scanout()) {native_legacy_transfer_observed(kIOReturnNotReady,request,0);return;}
            if(!native_legacy_fill_pixels(''', '''            if(!native_legacy_prepare_scanout()) {native_legacy_transfer_observed(kIOReturnNotReady,request,0);return;}
            if(latestOn()) {   // guamp latest-frame-wins (canvas variant section 10): the copy thread fills it in its turn
                const GuampFrame frame={nullptr,0,0,blankLength,blank.v_rowBytes,bw,bh,bw,bh,false,true};
                latestRetire(request,frame);return;
            }
            if(!native_legacy_fill_pixels('''))
# 11. Defaults (owner 28/10, ARGOMENTI-PREDEFINITI-20261028): the display arguments of the guamp boot line are code defaults;
#     an explicit key still wins (offline-legacy-*=0 turns one off).
for _key, _value in (('offline-legacy-pm', 1), ('offline-legacy-register', 1), ('offline-legacy-transfer', 1),
                     ('offline-legacy-validate', 1)):
    EDITS.append(('unsigned enabled=0;PE_parse_boot_argn("%s",&enabled,sizeof(enabled));' % _key,
                  'unsigned enabled=%u;PE_parse_boot_argn("%s",&enabled,sizeof(enabled));   // default %u (guamp 28/10)' % (_value, _key, _value)))
EDITS.append(('''        unsigned enabled=0;
        PE_parse_boot_argn("offline-legacy-gop",&enabled,sizeof(enabled));''', '''        unsigned enabled=1;   // default 1 (guamp 28/10)
        PE_parse_boot_argn("offline-legacy-gop",&enabled,sizeof(enabled));'''))
EDITS.append(('''        unsigned phase=0;PE_parse_boot_argn("offline-legacy-phase",&phase,sizeof(phase));
        if(enabled!=1 || phase!=4''', '''        unsigned phase=4;PE_parse_boot_argn("offline-legacy-phase",&phase,sizeof(phase));   // default 4 (guamp 28/10)
        if(enabled!=1 || phase!=4'''))
EDITS.append(('''        unsigned retire=0,cpuPresent=0;
        PE_parse_boot_argn("offline-legacy-retire",&retire,sizeof(retire));''', '''        unsigned retire=0,cpuPresent=1;   // offline-legacy-cpu-present default 1 (guamp 28/10)
        PE_parse_boot_argn("offline-legacy-retire",&retire,sizeof(retire));'''))
def apply(data):
    digest = hashlib.sha256(data).hexdigest()
    if PATCHED_SHA256 and digest == PATCHED_SHA256: return data
    assert digest == ORIGINAL_SHA256, 'Refuse an unreviewed NativeLegacyDisplay revision'
    text = data.decode()
    for old,new in EDITS:
        assert text.count(old) == 1, 'canvas variant: anchor not unique: '+old[:80]
        text = text.replace(old,new)
    result = text.encode()
    assert PATCHED_SHA256 is None or hashlib.sha256(result).hexdigest() == PATCHED_SHA256, 'canvas variant result differs'
    return result
if __name__ == '__main__':
    path = Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).resolve().parents[1]/'drivers/NativeMobileDisplay/NativeLegacyDisplay.cpp'
    path.write_bytes(apply(path.read_bytes()))
    print('NativeLegacyDisplay: verified guamp canvas variant (guamp-canvas-width/-height; guamp defaults of 28/10)')

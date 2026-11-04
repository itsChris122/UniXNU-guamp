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
Section 12 (guamp-compose, default 1): swaps with several layers (a video in a page) or a partial rectangle, which froze the screen
when refused, are composed in software.
Section 14 (frame pacing): the refresh tick (sheng-vsync) defaults to 60 Hz and follows the panel's INTF_1 counters
(guamp-vsync-hw); swap rate, swap -> completion latency and vsync messages are counted for the 'GuampPacing:' line.
"""
from pathlib import Path
import hashlib,sys
ORIGINAL_SHA256 = '09d7811db8fc6b659e12a05690867326e851f0d11bec7ffbb36b72c33ea035ca'
PATCHED_SHA256 = '226875923013958521cab8cd53f56ca41a13f88dace317a1a9a87da1612f7cf9'
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
    void latestStartCpu() {guamp_copy_split[8]=guamp_copy_split[8]+1;thread_call_enter(latestCall);}
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
# 12. Scanout hook (drivers/NativeEmmc/GuampScanout.inc; boot-args guamp-copy-diff, guamp-scanout-flip, guamp-copy-neon, all off by
#     default). The copy thread hands every frame to guamp_scanout_present() first: 1 drawn there (only the changed bands, or into
#     the second buffer of the double-buffered scanout), 0 not asked (weak default here, kernels without it), -1 refused (copied here
#     as before). A blank frame tells it first (guamp_scanout_external) so the GOP frame is the one shown and is redrawn whole.
EDITS.append(('volatile uint64_t guamp_copy_latest[10];}',
              'volatile uint64_t guamp_copy_latest[10];\n'
              '__attribute__((weak)) int guamp_scanout_present(volatile void *,uint64_t,uint64_t,uint64_t,const void *,uint64_t,uint64_t,\n'
              '    uint32_t,uint32_t,uint32_t,uint32_t,const uint32_t *,const uint32_t *){return 0;}\n'
              '__attribute__((weak)) void guamp_scanout_external(void){}}'))
EDITS.append((r'''            ok=(frame.scaled
                ?guampScalePixels(''', r'''            const bool tapsOk=!frame.scaled || (scaler.line && scaler.width==frame.width && scaler.height==frame.height &&
                                                scaler.sourceWidth==frame.offerWidth && scaler.sourceHeight==frame.offerHeight);
            const int hooked=!tapsOk?-1:guamp_scanout_present(reinterpret_cast<volatile void *>(bootMapping->getVirtualAddress()),
                bootMapping->getLength(),frame.rowBytes,bootMapping->getPhysicalAddress(),reinterpret_cast<const void *>(source->getVirtualAddress()),
                source->getLength(),frame.stride,frame.offerWidth,frame.offerHeight,frame.width,frame.height,
                frame.scaled?scaler.columns:nullptr,frame.scaled?scaler.rows:nullptr);
            ok=hooked>0 || (frame.scaled
                ?guampScalePixels('''))
EDITS.append(('''        if(frame.blank) {
            if(bootMapping && native_legacy_fill_pixels(''', '''        if(frame.blank) {
            guamp_scanout_external();   // section 12: the GOP frame is shown again before it is filled
            if(bootMapping && native_legacy_fill_pixels('''))
# 13. Software composition of multi-layer swaps (guamp-compose, default 1; 0 = the previous single-layer behaviour). A video in a page
#     (not full screen) makes backboardd present it as its own hardware layer: the swaps carry the page and the video in layers 1 and 2
#     with layer 0 disabled (state 7/1/0), which the single-layer validation refused (reason 2) for every frame, so the screen froze
#     (fin29/fin30/fin34). Request fields, read from the original swap_submit (kernelcache 0xfffffff00678f224: the user's 0x508-byte
#     IOMFBSwapRec at +0x38 into the request) and IOMobileFramebufferSwapSetLayer (dyld cache 0x1b63d2824), per layer i of 4:
#       src rect (x, y, w, h; surface pixels)  17 + 16 i      <- user +0xac     dst rect (display pixels)  81 + 16 i  <- user +0x10c
#       transform & 7                         580 + 4 i      <- user +0x15c    surface (IOSurface *)   2624 + 8 i  (updated layers)
#       updated / disabled / background       792 / 796 / 800                  options 804..812 (must be 0), display size 816/820.
#     A swap carries only the layers whose updated bit is set: the others keep what the previous swap set, as on the display hardware,
#     so the layers persist here (released when a later swap disables them or a single-layer/blank swap replaces the screen).
#     Layer 0 is the bottom. Each frame is composed by the copy thread (after its GPU wait) into a cached buffer of the panel size and
#     then copied to the scanout: premultiplied BGRA (bilinear, guamp-canvas-filter) with source-over, and biplanar 4:2:0 video ('420v'
#     video range, '420f' full range; BT.709 from 720 lines, else BT.601; nearest sample). IOSurface plane table (getters at
#     0xfffffff005b9eae0..0xfffffff005b9f3ac): array at +0x180, count +0x188, 56-byte entries: width +0, height +8, offset +0x10 (+0x18
#     the same for plain planes), row bytes +0x20, size +0x28. Other formats and transformed layers are skipped (counted, the format
#     logged) instead of refusing the swap. Composed swaps need latest-frame-wins (section 10, the default).
#     guamp_compose: [0] composed swaps accepted, [1] composed frames handed over, [2] frames composed, [3] compose failures, [4] BGRA
#     layers drawn, [5] 4:2:0 layers drawn, [6] layers skipped: format, [7] skipped: transform, [8] skipped: geometry, [9] last skipped
#     format, [10] compose ticks, [11] map failures, [12] swaps the composition refused too, [13] background fills, [14] most layers in a
#     frame, [15] 4:2:0 planes whose +0x10 and +0x18 offsets differ, [16] composed swaps without a drawable layer (retired, the screen
#     keeps the previous frame instead of turning black: fin37d showed only such swaps during a video), [17] requests dumped.
#     guamp-compose-dump=<n> (default 6, at most 64): the first n composed requests are logged raw ('GuampDisplay: compose dump'):
#     the key fields, then hex rows of bytes 0x000-0x160 (rects and regions), 0x1c0-0x260 (counts, per-layer words, transforms),
#     0x300-0x340 (masks, options, size) and 0xa40-0xa60 (surfaces), to check the layout against the real requests.
#     Speed (fin42: 34-113 ms per composed frame, the video stalled): the compose buffer keeps the last composed picture and only the
#     part that changed is redrawn and copied. Every layer a swap updates gets a new generation; the copy thread compares the frame's
#     layers (generation, destination) with what the buffer holds, so coalesced frames lose nothing, and redraws only the union of
#     the old and new rectangles of the changed layers (a video frame: its own rectangle, the page under it is not rescaled). A full
#     redraw (first frame, another frame took the scanout, background change) scales a full-screen page with the canvas scaler. The
#     result goes to the scanout hook (section 12, whole buffer) or, without it, only the changed rectangle is copied.
#     [18] BGRA layer ticks, [19] 4:2:0 layer ticks, [20] scanout copy ticks, [21] full redraws, [22] partial redraws, [23] frames
#     with nothing changed (no copy). guamp_compose_layer[i]: format, planes, surface width, height, src
#     x, y, w, h, dst x, y, w, h, transform, result of the last swap that updated layer i (0 drawable, 1 format, 2 transform, 3 geometry,
#     4 no IOSurface, 5 disabled).
EDITS.append(('''struct GuampFrame {
    IOMemoryDescriptor *memory;uint64_t stride,allocation,length,rowBytes;
    uint32_t width,height,offerWidth,offerHeight;bool scaled,blank;
};
''', r'''// guamp composition (scripts/guamp-canvas-variant.py section 12). The text between the two markers is the host-tested core
// (drivers/NativeEmmc/tests/compose_host.cpp extracts it): pure integer code, no kernel calls.
// guamp-compose-core-begin
struct GuampComposeLayer {
    const unsigned char *base;uint64_t bytes;   // the mapped surface (its whole allocation)
    uint32_t kind;                              // 0 premultiplied BGRA, 1 biplanar 4:2:0 video range, 2 the same in full range
    uint32_t matrix;                            // 4:2:0: 0 BT.601, 1 BT.709
    uint32_t width,height;                      // surface (luma plane) size
    uint64_t offset,stride;                     // first plane: byte offset and row bytes
    uint64_t chromaOffset,chromaStride;         // 4:2:0: the interleaved Cb Cr plane
    uint32_t sx,sy,sw,sh;                       // source rectangle, surface pixels
    int64_t dx,dy;uint32_t dw,dh;               // destination rectangle, output pixels (may reach past the output)
    unsigned filter;                            // BGRA: 1 bilinear, 0 nearest (4:2:0 always samples the nearest)
    bool blend;                                 // BGRA: premultiplied source-over instead of replacing
};
// One output sample: source index << 9 | weight of the next source sample (0..256), centres aligned.
static inline uint32_t guampComposeTap(uint32_t i,uint32_t outCount,uint32_t inCount,unsigned filter) {
    uint64_t position=(((uint64_t)(2*i+1)*inCount)<<16)/(2ull*outCount);
    position=position>=32768?position-32768:0;
    uint32_t index=(uint32_t)(position>>16),weight=(uint32_t)(((position&0xffff)+128)>>8);
    if(!filter){if(weight>=128)index=index+1;weight=0;}
    if(index>=inCount-1){index=inCount-1;weight=0;}
    return index<<9|weight;
}
static inline uint32_t guampComposeLerp(uint32_t a,uint32_t b,uint32_t w) {
    const uint32_t v=256-w;
    const uint32_t rb=((((a&0x00ff00ffU)*v+(b&0x00ff00ffU)*w)+0x00800080U)>>8)&0x00ff00ffU;
    const uint32_t ga=((((a>>8)&0x00ff00ffU)*v+((b>>8)&0x00ff00ffU)*w)+0x00800080U)&0xff00ff00U;
    return rb|ga;
}
// Premultiplied source over destination, per channel d * (255 - a) / 255 rounded, plus the source, saturated.
static inline uint32_t guampComposeOver(uint32_t s,uint32_t d) {
    const uint32_t a=s>>24;
    if(a==255)return s;
    if(!s)return d;
    const uint32_t k=255-a;
    uint32_t rb=(d&0x00ff00ffU)*k+0x00800080U;rb=((rb+((rb>>8)&0x00ff00ffU))>>8)&0x00ff00ffU;
    uint32_t ga=((d>>8)&0x00ff00ffU)*k+0x00800080U;ga=((ga+((ga>>8)&0x00ff00ffU))>>8)&0x00ff00ffU;
    rb+=s&0x00ff00ffU;ga+=(s>>8)&0x00ff00ffU;
    rb|=(rb&0x01000100U)-((rb&0x01000100U)>>8);ga|=(ga&0x01000100U)-((ga&0x01000100U)>>8);
    return (rb&0x00ff00ffU)|((ga&0x00ff00ffU)<<8);
}
static inline uint32_t guampComposeClamp(int32_t v) {return v<0?0:v>255?255:(uint32_t)v;}
// Y'CbCr to BGRA (8.8 fixed point): luma gain, Cr to R, Cb to G, Cr to G, Cb to B, luma offset. 601/709 video range, 601/709 full.
static const int32_t kGuampComposeYuv[4][6]={{298,409,100,208,516,16},{298,459,55,136,541,16},{256,359,88,183,454,0},{256,403,48,120,475,0}};
static inline uint32_t guampComposeYuv(uint32_t y,uint32_t cb,uint32_t cr,const int32_t *k) {
    const int32_t c=((int32_t)y-k[5])*k[0],d=(int32_t)cb-128,e=(int32_t)cr-128;
    const int32_t r=(c+k[1]*e+128)>>8,g=(c-k[2]*d-k[3]*e+128)>>8,b=(c+k[4]*d+128)>>8;
    return 0xff000000U|guampComposeClamp(r)<<16|guampComposeClamp(g)<<8|guampComposeClamp(b);
}
// Every byte the layer reads lies in its surface.
static inline bool guampComposeValid(const GuampComposeLayer &l) {
    if(!l.base || l.kind>2 || l.matrix>1 || !l.sw || !l.sh || !l.dw || !l.dh || l.sw>16384 || l.sh>16384 || l.dw>16384 || l.dh>16384 ||
       l.width>16384 || l.height>16384 || (uint64_t)l.sx+l.sw>l.width || (uint64_t)l.sy+l.sh>l.height || !l.stride || l.stride>(1u<<20) ||
       l.offset>l.bytes)return false;
    const uint64_t pixel=l.kind?1:4,rows=(uint64_t)l.sy+l.sh,right=((uint64_t)l.sx+l.sw)*pixel;
    if(l.stride<right || (l.bytes-l.offset)<l.stride*(rows-1)+right)return false;
    if(!l.kind)return !(l.offset&3) && !(l.stride&3);
    const uint64_t chromaRows=(rows+1)/2,chromaRight=(((uint64_t)l.sx+l.sw+1)/2)*2;
    return l.chromaStride && l.chromaStride<=(1u<<20) && l.chromaStride>=chromaRight && l.chromaOffset<=l.bytes &&
           (l.bytes-l.chromaOffset)>=l.chromaStride*(chromaRows-1)+chromaRight;
}
// Draws one layer into the width x height output, only inside the clip rectangle [cx0,cx1) x [cy0,cy1) (the part of the screen
// that changed). taps: at least visible columns + rows words; line: (sw + 1) * 4 bytes for BGRA.
// Returns the output pixels written (0 when the layer is off the output or the clip), -1 when nothing was drawn (invalid layer).
static inline int64_t guampComposeDraw(uint32_t *out,uint32_t width,uint32_t height,const GuampComposeLayer &l,
    uint32_t *taps,uint64_t tapWords,unsigned char *line,uint64_t lineBytes,int64_t cx0,int64_t cy0,int64_t cx1,int64_t cy1) {
    if(!out || !width || !height || !taps || !guampComposeValid(l))return -1;
    if(cx0<0)cx0=0;
    if(cy0<0)cy0=0;
    if(cx1>(int64_t)width)cx1=width;
    if(cy1>(int64_t)height)cy1=height;
    int64_t x0=l.dx<0?0:l.dx,y0=l.dy<0?0:l.dy;
    int64_t x1=l.dx+(int64_t)l.dw>(int64_t)width?(int64_t)width:l.dx+(int64_t)l.dw;
    int64_t y1=l.dy+(int64_t)l.dh>(int64_t)height?(int64_t)height:l.dy+(int64_t)l.dh;
    if(x0<cx0)x0=cx0;
    if(y0<cy0)y0=cy0;
    if(x1>cx1)x1=cx1;
    if(y1>cy1)y1=cy1;
    if(x0>=x1 || y0>=y1)return 0;
    const uint32_t cols=(uint32_t)(x1-x0),rows=(uint32_t)(y1-y0);
    const bool bgra=!l.kind;
    if(tapWords<(uint64_t)cols+rows || (bgra && (!line || lineBytes<((uint64_t)l.sw+1)*4)))return -1;
    const unsigned filter=bgra?l.filter:0;
    uint32_t *columnTaps=taps,*rowTaps=taps+cols;
    for(uint32_t c=0;c<cols;++c)columnTaps[c]=guampComposeTap((uint32_t)(x0-l.dx)+c,l.dw,l.sw,filter);
    for(uint32_t r=0;r<rows;++r)rowTaps[r]=guampComposeTap((uint32_t)(y0-l.dy)+r,l.dh,l.sh,filter);
    const int32_t *k=kGuampComposeYuv[(l.kind==2?2:0)+l.matrix];
    const bool direct=l.sw==l.dw;   // one source column per output column: no column weights
    for(uint32_t r=0;r<rows;++r) {
        uint32_t *o=out+(uint64_t)(y0+r)*width+(uint64_t)x0;
        const uint32_t row=l.sy+(rowTaps[r]>>9),weight=rowTaps[r]&511;
        if(bgra) {
            const unsigned char *first=l.base+l.offset+(uint64_t)row*l.stride+(uint64_t)l.sx*4;
            const unsigned char *src=first;
            if(weight) {   // the tap guarantees row + 1 inside the rectangle
                const unsigned char *second=first+l.stride;
                for(uint32_t x=0;x<l.sw;++x) {
                    uint32_t a,b;__builtin_memcpy(&a,first+(uint64_t)x*4,4);__builtin_memcpy(&b,second+(uint64_t)x*4,4);
                    const uint32_t v=guampComposeLerp(a,b,weight);__builtin_memcpy(line+(uint64_t)x*4,&v,4);
                }
                src=line;
            }
            for(uint32_t c=0;c<cols;++c) {
                const uint32_t tap=columnTaps[c],at=(tap>>9)*4,w=direct?0:(tap&511);
                uint32_t s;__builtin_memcpy(&s,src+at,4);
                if(w){uint32_t t;__builtin_memcpy(&t,src+at+4,4);s=guampComposeLerp(s,t,w);}
                o[c]=l.blend?guampComposeOver(s,o[c]):s;
            }
        } else {
            const unsigned char *luma=l.base+l.offset+(uint64_t)row*l.stride;
            const unsigned char *chroma=l.base+l.chromaOffset+(uint64_t)(row/2)*l.chromaStride;
            for(uint32_t c=0;c<cols;++c) {
                const uint32_t column=l.sx+(columnTaps[c]>>9),pair=column&~1u;
                o[c]=guampComposeYuv(luma[column],chroma[pair],chroma[pair+1],k);
            }
        }
    }
    return (int64_t)cols*rows;
}
// Union of a destination rectangle (clipped to the output) into the box r = {x0, y0, x1, y1} (empty when x0 >= x1).
static inline void guampComposeUnion(int64_t *r,int64_t dx,int64_t dy,uint64_t dw,uint64_t dh,uint32_t width,uint32_t height) {
    int64_t x0=dx<0?0:dx,y0=dy<0?0:dy,x1=dx+(int64_t)dw,y1=dy+(int64_t)dh;
    if(x1>(int64_t)width)x1=width;
    if(y1>(int64_t)height)y1=height;
    if(x0>=x1 || y0>=y1)return;
    if(r[0]>=r[2] || r[1]>=r[3]){r[0]=x0;r[1]=y0;r[2]=x1;r[3]=y1;return;}
    if(x0<r[0])r[0]=x0;
    if(y0<r[1])r[1]=y0;
    if(x1>r[2])r[2]=x1;
    if(y1>r[3])r[3]=y1;
}
static inline void guampComposeFill(uint32_t *out,uint64_t pixels,uint32_t value) {
    const uint64_t pair=(uint64_t)value<<32|value;uint64_t i=0;
    if(!((uintptr_t)out&7))for(;i+2<=pixels;i+=2)*reinterpret_cast<uint64_t *>(out+i)=pair;
    for(;i<pixels;++i)out[i]=value;
}
// guamp-compose-core-end
extern "C" {volatile uint64_t guamp_compose[24];volatile uint32_t guamp_compose_layer[4][14];}
// The layers of one composed frame, shared by the frames that show them (latest-frame-wins keeps the one on screen): each surface's
// memory stays retained until the last frame holding the set lets it go.
struct GuampLayerSource {IOMemoryDescriptor *memory;uint64_t allocation;GuampComposeLayer layer;uint32_t slot,gen;};
struct GuampLayers {
    uint32_t refs,count,background;bool fill;uint64_t serial;
    GuampLayerSource source[4];
    void retain() {__atomic_add_fetch(&refs,1,__ATOMIC_RELAXED);}
    void release() {
        if(__atomic_sub_fetch(&refs,1,__ATOMIC_ACQ_REL))return;
        for(uint32_t i=0;i<count;++i)OSSafeReleaseNULL(source[i].memory);
        IOFree(this,sizeof(*this));
    }
};
struct GuampFrame {
    IOMemoryDescriptor *memory;uint64_t stride,allocation,length,rowBytes;
    uint32_t width,height,offerWidth,offerHeight;bool scaled,blank;
    GuampLayers *layers;   // section 12: a composed frame (memory is null)
};
'''))
EDITS.append(('''                                    scaled?sourceWidth:width,scaled?sourceHeight:height,scaled,false};''',
              '''                                    scaled?sourceWidth:width,scaled?sourceHeight:height,scaled,false,nullptr};'''))
EDITS.append(('const GuampFrame frame={nullptr,0,0,blankLength,blank.v_rowBytes,bw,bh,bw,bh,false,true};',
              'const GuampFrame frame={nullptr,0,0,blankLength,blank.v_rowBytes,bw,bh,bw,bh,false,true,nullptr};'))
EDITS.append(('''    void latestRetain(const GuampFrame &frame) {if(frame.memory)frame.memory->retain();}
    void latestRelease(const GuampFrame &frame) {if(frame.memory)frame.memory->release();}''',
              '''    void latestRetain(const GuampFrame &frame) {if(frame.memory)frame.memory->retain();if(frame.layers)frame.layers->retain();}
    void latestRelease(const GuampFrame &frame) {if(frame.memory)frame.memory->release();if(frame.layers)frame.layers->release();}'''))
EDITS.append(('''    bool latestCopyOnce(const GuampFrame &frame) {
        IOMemoryMap *source=guampSourceMap(frame.memory,frame.allocation);''',
              '''    bool latestCopyOnce(const GuampFrame &frame) {
        if(frame.layers)return composeOnce(frame);   // section 12
        IOMemoryMap *source=guampSourceMap(frame.memory,frame.allocation);'''))
EDITS.append(('        } else if(frame.memory) {\n', '        } else if(frame.memory || frame.layers) {\n'))
EDITS.append(('        latest.clear(*this);\n', '        latest.clear(*this);\n        freeCompose();\n'))
# Two layers per frame plus the single-layer frames: keep the mappings of 8 surfaces.
EDITS.append(('GuampMapSlot mapSlots[4]={};', 'GuampMapSlot mapSlots[8]={};'))
EDITS.append(('    void removeValidated(const OriginalLegacySwapRequest *request) {for(auto &entry: validated)if(entry==request)entry=nullptr;}',
              '    void removeValidated(const OriginalLegacySwapRequest *request) {for(auto &entry: validated)if(entry==request)entry=nullptr;\n'
              '        for(auto &entry: composedValidated)if(entry.request==request)entry={};}'))
EDITS.append(('transferred=false;for(auto &entry: validated)entry=nullptr;gpuRequest=nullptr;',
              'transferred=false;for(auto &entry: validated)entry=nullptr;for(auto &entry: composedValidated)entry={};gpuRequest=nullptr;'))
EDITS.append(('    // Section 10: latest-frame-wins.\n', r'''    // Section 12: software composition of multi-layer swaps.
    // The layer masks are taken when the swap is validated: by the time the queue reaches it the family has rewritten the word at 796
    // (fin40b dumps: 7/1 at validation, 7/7 at the kick, every layer read as disabled and nothing drawn).
    struct ComposedEntry {const OriginalLegacySwapRequest *request;uint32_t updated,disabled,background;};
    ComposedEntry composedValidated[8]={};
    const ComposedEntry *composedEntry(const OriginalLegacySwapRequest *request) const {
        if(!request)return nullptr;
        for(auto &entry: composedValidated)if(entry.request==request)return &entry;
        return nullptr;
    }
    void addComposed(const OriginalLegacySwapRequest *request,const unsigned char *bytes) {
        ComposedEntry e={request,0,0,0};memcpy(&e.updated,bytes+792,4);memcpy(&e.disabled,bytes+796,4);memcpy(&e.background,bytes+800,4);
        for(auto &entry: composedValidated)if(entry.request==request){entry=e;return;}
        for(auto &entry: composedValidated)if(!entry.request){entry=e;return;}
        composedValidated[0]=e;
    }
    int composeWanted=-1;
    GuampLayerSource composeCurrent[4]={};uint32_t composeBackground=0xff000000U;
    void *composeBuffer=nullptr;vm_size_t composeBufferSize=0;
    void *composeScratch=nullptr;vm_size_t composeScratchSize=0;
    bool composeOn() {
        if(composeWanted<0){unsigned v=1;PE_parse_boot_argn("guamp-compose",&v,sizeof v);composeWanted=v!=0;}
        return composeWanted==1;
    }
    // The persistent layers go (a single-layer or blank swap replaced the screen, or teardown).
    void composeReset() {
        for(auto &s: composeCurrent){OSSafeReleaseNULL(s.memory);s={};}
        composeBackground=0xff000000U;
        __atomic_store_n(&composeStale,true,__ATOMIC_RELEASE);   // the scanout no longer shows the compose buffer
    }
    void freeCompose() {
        composeReset();
        if(composeBuffer){IOFreeAligned(composeBuffer,composeBufferSize);composeBuffer=nullptr;composeBufferSize=0;}
        if(composeScratch){IOFree(composeScratch,composeScratchSize);composeScratch=nullptr;composeScratchSize=0;}
    }
    // Layer i of the request, read from the request and its IOSurface. 0 drawable (out filled, memory retained when out is set),
    // 1 unsupported format, 2 transformed, 3 bad geometry, 4 no IOSurface.
    unsigned composeLayer(const unsigned char *bytes,unsigned i,uint32_t displayWidth,uint32_t displayHeight,GuampLayerSource *out) {
        volatile uint32_t *note=guamp_compose_layer[i];
        OSObject *surface=nullptr;memcpy(&surface,bytes+2624+8*i,sizeof(surface));
        if(!surface || !surface->metaCast("IOSurface"))return 4;
        IOMemoryDescriptor *memory=native_legacy_surface_descriptor(surface);
        if(!memory || !memory->metaCast("IOMemoryDescriptor"))return 4;
        const uint32_t format=native_legacy_surface_format(surface),planes=native_legacy_surface_planes(surface);
        const uint64_t width=native_legacy_surface_width(surface),height=native_legacy_surface_height(surface);
        const uint64_t allocation=native_legacy_surface_allocation(surface);
        uint32_t source[4]={},target[4]={},transform=0;
        memcpy(source,bytes+17+16*i,sizeof(source));memcpy(target,bytes+81+16*i,sizeof(target));memcpy(&transform,bytes+580+4*i,4);
        note[0]=format;note[1]=planes;note[2]=(uint32_t)width;note[3]=(uint32_t)height;
        for(unsigned j=0;j<4;++j){note[4+j]=source[j];note[8+j]=target[j];}
        note[12]=transform;
        GuampComposeLayer l={};
        if(width>16384 || height>16384 || !allocation || allocation>memory->getLength())return 3;
        if(nativeBgra(format,planes)) {
            l.kind=0;l.offset=0;l.stride=native_legacy_surface_stride(surface);
        } else if((format==0x34323076 || format==0x34323066) && planes==2) {   // '420v', '420f'
            const unsigned char *object=reinterpret_cast<const unsigned char *>(surface);
            uint64_t table=0;uint32_t entries=0;memcpy(&table,object+0x180,8);memcpy(&entries,object+0x188,4);
            if(!table || (table>>48)!=0xffff || entries<2)return 1;
            uint64_t luma[6]={},chroma[6]={};   // width, height, offset, second offset, row bytes, size
            memcpy(luma,reinterpret_cast<const void *>(table),sizeof(luma));memcpy(chroma,reinterpret_cast<const void *>(table+56),sizeof(chroma));
            if(luma[2]!=luma[3] || chroma[2]!=chroma[3])guamp_compose[15]=guamp_compose[15]+1;
            if(luma[0]<width || luma[1]<height || 2*chroma[0]<width || 2*chroma[1]<height ||
               luma[2]>allocation || luma[5]>allocation-luma[2] || chroma[2]>allocation || chroma[5]>allocation-chroma[2])return 3;
            l.kind=format==0x34323066?2:1;l.matrix=height>=720?1:0;
            l.offset=luma[2];l.stride=luma[4];l.chromaOffset=chroma[2];l.chromaStride=chroma[4];
        } else {guamp_compose[9]=format;return 1;}
        if(transform&7)return 2;
        l.width=(uint32_t)width;l.height=(uint32_t)height;
        l.sx=source[0];l.sy=source[1];l.sw=source[2];l.sh=source[3];
        // Destination in display coordinates (signed: the client clips, this does too), mapped to the scanout when they differ.
        int64_t dx=(int32_t)target[0],dy=(int32_t)target[1];uint64_t dw=target[2],dh=target[3];
        if(displayWidth!=gopWidth || displayHeight!=gopHeight) {
            dx=dx*(int64_t)gopWidth/(int64_t)displayWidth;dy=dy*(int64_t)gopHeight/(int64_t)displayHeight;
            dw=dw*gopWidth/displayWidth;dh=dh*gopHeight/displayHeight;
        }
        if(dw>16384 || dh>16384 || dx>=(int64_t)gopWidth || dy>=(int64_t)gopHeight || dx+(int64_t)dw<=0 || dy+(int64_t)dh<=0)return 3;
        l.dx=dx;l.dy=dy;l.dw=(uint32_t)dw;l.dh=(uint32_t)dh;l.filter=canvasFilter;
        l.base=reinterpret_cast<const unsigned char *>(1);l.bytes=allocation;   // the geometry against the allocation; mapped later
        const bool valid=guampComposeValid(l);l.base=nullptr;l.bytes=0;
        if(!valid)return 3;
        if(out){memory->retain();out->memory=memory;out->allocation=allocation;out->layer=l;}
        return 0;
    }
    // A swap the single-layer path refused (reason 2 or 3): known layer bits, no options, every updated enabled layer an IOSurface.
    bool composeAccepts(const unsigned char *bytes) {
        if(!composeOn() || !logicalPM || !primary || !bootMapping || !gopWidth || !gopHeight || !latestOn())return false;
        auto word=[&](unsigned offset){uint32_t value=0;memcpy(&value,bytes+offset,4);return value;};
        const uint32_t updated=word(792),disabled=word(796);
        if(((updated|disabled)&~0x8000000fU) || (disabled&~updated) || !(updated&0xfU))return false;
        for(unsigned offset=804;offset<=812;offset+=4)if(word(offset))return false;
        for(unsigned i=0;i<4;++i) {
            if(!(updated&~disabled&(1u<<i)))continue;
            OSObject *surface=nullptr;memcpy(&surface,bytes+2624+8*i,sizeof(surface));
            if(!surface || !surface->metaCast("IOSurface"))return false;
        }
        return true;
    }
    // guamp-compose-dump: the first composed requests, raw.
    int composeDumpLeft=-1;
    void composeDumpRows(unsigned n,const unsigned char *bytes,unsigned from,unsigned to) {
        for(unsigned at=from;at<to;at+=32) {
            char hex[32*3+1];unsigned k=0;
            for(unsigned j=0;j<32;++j)k+=snprintf(hex+k,sizeof(hex)-k,"%02x%s",bytes[at+j],j==31?"":((j&3)==3?" ":""));
            IOLog("GuampDisplay: compose dump %u +0x%03x: %s\n",n,at,hex);
        }
    }
    void composeDump(const unsigned char *bytes,const ComposedEntry &validatedMasks) {
        if(composeDumpLeft<0){unsigned v=6;PE_parse_boot_argn("guamp-compose-dump",&v,sizeof v);composeDumpLeft=(int)(v>64?64:v);}
        if(!composeDumpLeft)return;
        composeDumpLeft=composeDumpLeft-1;
        const unsigned n=(unsigned)guamp_compose[17];guamp_compose[17]=guamp_compose[17]+1;
        uint32_t w[12]={};memcpy(w,bytes+792,sizeof(w));   // 792..836
        uint64_t surfaces[4]={};memcpy(surfaces,bytes+2624,sizeof(surfaces));
        const char *names[4]={"-","-","-","-"};
        for(unsigned i=0;i<4;++i)if(surfaces[i]) {
            const OSMetaClass *meta=reinterpret_cast<const OSObject *>(surfaces[i])->getMetaClass();
            names[i]=meta && meta->getClassName()?meta->getClassName():"?";
        }
        IOLog("GuampDisplay: compose dump %u: validated updated %x disabled %x background %x; now updated %x disabled %x background %x options %x %x %x size %ux%u; words 824.. %x %x %x %x; "
              "surfaces %llx %llx %llx %llx (%s %s %s %s)\n",n,validatedMasks.updated,validatedMasks.disabled,validatedMasks.background,w[0],w[1],w[2],w[3],w[4],w[5],w[6],w[7],w[8],w[9],w[10],w[11],
              surfaces[0],surfaces[1],surfaces[2],surfaces[3],names[0],names[1],names[2],names[3]);
        composeDumpRows(n,bytes,0x000,0x160);composeDumpRows(n,bytes,0x1c0,0x260);
        composeDumpRows(n,bytes,0x300,0x340);composeDumpRows(n,bytes,0xa40,0xa60);
    }
    // The validated head of a composed swap: its layers update the persistent ones, a snapshot of them becomes the frame.
    void kickComposed(const OriginalLegacySwapRequest *request,const ComposedEntry &validatedMasks) {
        const unsigned char *bytes=reinterpret_cast<const unsigned char *>(request);
        PE_Video video={};
        const uint64_t length=(uint64_t)gopWidth*gopHeight*4;
        if(PE_current_console(&video) || video.v_baseAddr!=bootMapping->getPhysicalAddress() || video.v_width!=gopWidth ||
           video.v_height!=gopHeight || video.v_depth!=32 || video.v_rotate ||
           !native_legacy_surface_bounds(gopWidth,gopHeight,video.v_rowBytes,bootMapping->getLength(),bootMapping->getLength())) {
            native_legacy_transfer_observed(kIOReturnNotReady,request,0);return;
        }
        if(!native_legacy_prepare_scanout()) {native_legacy_transfer_observed(kIOReturnNotReady,request,0);return;}
        uint32_t displayWidth=0,displayHeight=0;getDisplayDimensions(&displayWidth,&displayHeight);
        if(!displayWidth || !displayHeight){native_legacy_transfer_observed(kIOReturnNotReady,request,0);return;}
        const uint32_t updated=validatedMasks.updated,disabled=validatedMasks.disabled,background=validatedMasks.background;
        for(unsigned i=0;i<4;++i) {
            const uint32_t bit=1u<<i;
            if(!(updated&bit))continue;
            OSSafeReleaseNULL(composeCurrent[i].memory);composeCurrent[i]={};
            if(disabled&bit){guamp_compose_layer[i][13]=5;continue;}
            const unsigned result=composeLayer(bytes,i,displayWidth,displayHeight,&composeCurrent[i]);
            composeGen[i]=composeGen[i]+1;composeCurrent[i].slot=i;composeCurrent[i].gen=composeGen[i];
            guamp_compose_layer[i][13]=result;
            if(result>=1 && result<=3)guamp_compose[5+result]=guamp_compose[5+result]+1;
        }
        if(updated&0x80000000U)composeBackground=0xff000000U|(background&0x00ffffffU);
        composeDump(bytes,validatedMasks);
        GuampLayers *set=static_cast<GuampLayers *>(IOMalloc(sizeof(GuampLayers)));
        if(!set){native_legacy_transfer_observed(kIOReturnNoMemory,request,0);return;}
        bzero(set,sizeof(*set));set->refs=1;set->background=composeBackground;set->fill=true;
        composeSerial=composeSerial+1;set->serial=composeSerial;
        for(unsigned i=0;i<4;++i) {
            if(!composeCurrent[i].memory)continue;
            GuampLayerSource &s=set->source[set->count];s=composeCurrent[i];s.memory->retain();s.layer.blend=true;
            if(!set->count && !(composeBackground&0x00ffffffU) && s.layer.dx<=0 && s.layer.dy<=0 &&
               s.layer.dx+(int64_t)s.layer.dw>=(int64_t)gopWidth && s.layer.dy+(int64_t)s.layer.dh>=(int64_t)gopHeight) {
                set->fill=false;s.layer.blend=false;   // the bottom layer covers the black background: replace, no fill
            }
            set->count=set->count+1;
        }
        if(set->count>guamp_compose[14])guamp_compose[14]=set->count;
        if(!set->count) {
            // Nothing to draw: retire the swap and leave the screen as it is (a black fill hid the page, fin37d).
            set->release();guamp_compose[16]=guamp_compose[16]+1;
            native_legacy_swap_stage(1);transferred=true;native_legacy_transfer_observed(kIOReturnSuccess,request,0);
            finishTransfer(request,true);return;
        }
        guamp_compose[1]=guamp_compose[1]+1;
        const GuampFrame frame={nullptr,0,0,length,video.v_rowBytes,gopWidth,gopHeight,gopWidth,gopHeight,false,false,set};
        latestRetire(request,frame);
    }
    // Work loop: layer generations and frame serials. Copy thread: what the compose buffer holds.
    uint32_t composeGen[4]={};uint64_t composeSerial=0;
    struct ComposeShown {bool present;uint32_t gen;int64_t dx,dy;uint32_t dw,dh;};
    ComposeShown composeShown[4]={};uint32_t composeShownBackground=0;bool composeShownFill=false;
    bool composeStale=true;uint64_t composeLastSerial=0;int64_t composeLastDirty[4]={};
    static uint64_t composeTicks() {uint64_t t;__asm__ volatile("isb\n\tmrs %0, cntpct_el0":"=r"(t));return t;}
    // Copy thread: the part of the screen that changed since the buffer was last composed (the same frame again, a re-copy after
    // the GPU wait or a trailing re-show, redraws the same part), then the layers inside it, then the scanout.
    bool composeOnce(const GuampFrame &frame) {
        GuampLayers *set=frame.layers;
        const uint64_t pixels=(uint64_t)frame.width*frame.height,need=pixels*4;
        if(!set || !bootMapping || !pixels)return false;
        bool full=__atomic_exchange_n(&composeStale,false,__ATOMIC_ACQ_REL);
        if(!composeBuffer || composeBufferSize<need) {
            if(composeBuffer)IOFreeAligned(composeBuffer,composeBufferSize);
            composeBufferSize=(vm_size_t)((need+16383)&~16383ull);composeBuffer=IOMallocAligned(composeBufferSize,16384);
            if(!composeBuffer){composeBufferSize=0;guamp_compose[3]=guamp_compose[3]+1;__atomic_store_n(&composeStale,true,__ATOMIC_RELEASE);return false;}
            full=true;
        }
        uint64_t widest=0;
        for(uint32_t i=0;i<set->count;++i)if(set->source[i].layer.sw>widest)widest=set->source[i].layer.sw;
        const uint64_t tapWords=(uint64_t)frame.width+frame.height,scratchNeed=tapWords*4+(widest+1)*4;
        if(!composeScratch || composeScratchSize<scratchNeed) {
            if(composeScratch)IOFree(composeScratch,composeScratchSize);
            composeScratchSize=(vm_size_t)scratchNeed;composeScratch=IOMalloc(composeScratchSize);
            if(!composeScratch){composeScratchSize=0;guamp_compose[3]=guamp_compose[3]+1;__atomic_store_n(&composeStale,true,__ATOMIC_RELEASE);return false;}
        }
        if(set->fill!=composeShownFill || (set->fill && set->background!=composeShownBackground))full=true;
        int64_t dirty[4]={0,0,0,0};
        if(full) {dirty[2]=frame.width;dirty[3]=frame.height;}
        else if(set->serial==composeLastSerial) {for(unsigned k=0;k<4;++k)dirty[k]=composeLastDirty[k];}
        else for(uint32_t slot=0;slot<4;++slot) {
            const GuampLayerSource *now=nullptr;
            for(uint32_t i=0;i<set->count;++i)if(set->source[i].slot==slot)now=&set->source[i];
            const ComposeShown &was=composeShown[slot];
            const bool same=now?(was.present && was.gen==now->gen && was.dx==now->layer.dx && was.dy==now->layer.dy &&
                                 was.dw==now->layer.dw && was.dh==now->layer.dh):!was.present;
            if(same)continue;
            if(was.present)guampComposeUnion(dirty,was.dx,was.dy,was.dw,was.dh,frame.width,frame.height);
            if(now)guampComposeUnion(dirty,now->layer.dx,now->layer.dy,now->layer.dw,now->layer.dh,frame.width,frame.height);
        }
        for(uint32_t slot=0;slot<4;++slot)composeShown[slot]={};
        for(uint32_t i=0;i<set->count;++i) {
            const GuampLayerSource &src=set->source[i];
            if(src.slot<4)composeShown[src.slot]={true,src.gen,src.layer.dx,src.layer.dy,src.layer.dw,src.layer.dh};
        }
        composeShownFill=set->fill;composeShownBackground=set->background;
        composeLastSerial=set->serial;for(unsigned k=0;k<4;++k)composeLastDirty[k]=dirty[k];
        if(dirty[0]>=dirty[2] || dirty[1]>=dirty[3]) {   // nothing changed: the scanout already shows this frame
            guamp_compose[23]=guamp_compose[23]+1;guamp_compose[2]=guamp_compose[2]+1;return true;
        }
        const bool whole=dirty[0]==0 && dirty[1]==0 && dirty[2]==(int64_t)frame.width && dirty[3]==(int64_t)frame.height;
        if(whole)guamp_compose[21]=guamp_compose[21]+1;else guamp_compose[22]=guamp_compose[22]+1;
        uint32_t *taps=static_cast<uint32_t *>(composeScratch);
        unsigned char *line=static_cast<unsigned char *>(composeScratch)+tapWords*4;
        uint32_t *out=static_cast<uint32_t *>(composeBuffer);
        const uint64_t composeStart=composeTicks();
        if(set->fill) {
            for(int64_t y=dirty[1];y<dirty[3];++y)
                guampComposeFill(out+(uint64_t)y*frame.width+(uint64_t)dirty[0],(uint64_t)(dirty[2]-dirty[0]),set->background);
            guamp_compose[13]=guamp_compose[13]+1;
        }
        for(uint32_t i=0;i<set->count;++i) {
            const GuampLayerSource &src=set->source[i];
            GuampComposeLayer l=src.layer;
            // Off the changed part: not even mapped.
            if(l.dx>=dirty[2] || l.dy>=dirty[3] || l.dx+(int64_t)l.dw<=dirty[0] || l.dy+(int64_t)l.dh<=dirty[1])continue;
            const uint64_t layerStart=composeTicks();
            IOMemoryMap *map=guampSourceMap(src.memory,src.allocation);
            if(!map || !map->getVirtualAddress() || map->getLength()<src.allocation) {
                OSSafeReleaseNULL(map);guamp_compose[11]=guamp_compose[11]+1;continue;
            }
            l.base=reinterpret_cast<const unsigned char *>(map->getVirtualAddress());l.bytes=src.allocation;
            int64_t drawn=-1;
            // A whole redraw of a full-screen page: the canvas scaler (two taps per pixel, 64-bit stores), as single-layer frames.
            if(whole && !l.kind && !l.blend && !l.sx && !l.sy && !l.dx && !l.dy && l.dw==frame.width && l.dh==frame.height &&
               (l.sw!=l.dw || l.sh!=l.dh) && guampComposeValid(l) &&
               // the tables the work loop built for single-layer canvas frames, only read here (never rebuilt from this thread)
               scaler.line && scaler.sourceWidth==l.sw && scaler.sourceHeight==l.sh && scaler.width==frame.width && scaler.height==frame.height &&
               guampScalePixels(reinterpret_cast<volatile void *>(out),need,(uint64_t)frame.width*4,l.base+l.offset,
                   l.bytes-l.offset,l.stride,scaler))drawn=(int64_t)pixels;
            if(drawn<0)drawn=guampComposeDraw(out,frame.width,frame.height,l,taps,tapWords,line,(widest+1)*4,dirty[0],dirty[1],dirty[2],dirty[3]);
            OSSafeReleaseNULL(map);
            const uint64_t layerTicks=composeTicks()-layerStart;
            if(l.kind)guamp_compose[19]=guamp_compose[19]+layerTicks;else guamp_compose[18]=guamp_compose[18]+layerTicks;
            if(drawn<0){guamp_compose[8]=guamp_compose[8]+1;continue;}
            if(l.kind)guamp_compose[5]=guamp_compose[5]+1;else guamp_compose[4]=guamp_compose[4]+1;
        }
        const uint64_t copyStart=composeTicks();
        guamp_compose[10]=guamp_compose[10]+(copyStart-composeStart);
        // Section 12 hook first (whole buffer: it finds the changed bands itself and may flip buffers), else the changed part only.
        const int hooked=guamp_scanout_present(reinterpret_cast<volatile void *>(bootMapping->getVirtualAddress()),bootMapping->getLength(),
            frame.rowBytes,bootMapping->getPhysicalAddress(),out,need,(uint64_t)frame.width*4,frame.width,frame.height,frame.width,
            frame.height,nullptr,nullptr);
        bool ok=hooked>0;
        if(!ok) {
            const uint64_t at=(uint64_t)dirty[1]*frame.rowBytes+(uint64_t)dirty[0]*4,from=((uint64_t)dirty[1]*frame.width+(uint64_t)dirty[0])*4;
            ok=at<bootMapping->getLength() && native_legacy_copy_pixels(
                reinterpret_cast<volatile void *>(bootMapping->getVirtualAddress()+at),bootMapping->getLength()-at,frame.rowBytes,
                reinterpret_cast<const unsigned char *>(out)+from,need-from,(uint64_t)frame.width*4,
                (uint32_t)(dirty[2]-dirty[0]),(uint32_t)(dirty[3]-dirty[1]))!=0;
        }
        __asm__ volatile("dsb sy" ::: "memory");
        const uint64_t copyEnd=composeTicks();
        guamp_compose[20]=guamp_compose[20]+(copyEnd-copyStart);
        if(ok){native_legacy_copy_timing_observed(copyEnd-copyStart,frame.length);guamp_compose[2]=guamp_compose[2]+1;}
        else {guamp_compose[3]=guamp_compose[3]+1;__atomic_store_n(&composeStale,true,__ATOMIC_RELEASE);}
        return ok;
    }
    // Section 10: latest-frame-wins.
'''))
# The composed path in the validator (after the single-layer and blank checks) and in the queue kick.
EDITS.append(('''        if(accepted)addValidated(request);else removeValidated(request);
        native_legacy_swap_validation_observed(accepted,reason,request,surface);return accepted;''',
              '''        // guamp composition (canvas variant section 12): what the single-layer path refused for its layers, geometry or format.
        // reason 4 is a single full-screen layer whose surface this port cannot draw directly (the boot compositor frame is an
        // AGX tile-compressed 'b3a8' BGRA, 2 planes): the composition skips the unsupported format and keeps the previous frame
        // instead of a hard reject, which the family logs as "verify_swap failed for swap id" once per swap (2111 at boot, fin43).
        bool composed=false;
        if(!accepted && enabled==1 && request && (reason==2 || reason==3 || reason==4)) {
            if(composeAccepts(bytes)){reason=6;accepted=composed=true;guamp_compose[0]=guamp_compose[0]+1;}
            else guamp_compose[12]=guamp_compose[12]+1;
        }
        if(accepted)addValidated(request);else removeValidated(request);
        if(composed)addComposed(request,bytes);
        native_legacy_swap_validation_observed(accepted,reason,request,surface);return accepted;'''))
EDITS.append(('''        if(!request || !isValidated(request))return;
''', '''        if(!request || !isValidated(request))return;
        // guamp composition (canvas variant section 12): a composed swap; any other frame replaces the persistent layers.
        if(const ComposedEntry *entry=composedEntry(request)){const ComposedEntry masks=*entry;kickComposed(request,masks);return;}
        composeReset();
'''))
# Section 13 speed: every other frame the copy thread puts on the scanout (single-layer copy, blank fill) makes the next composed
# frame a whole redraw (set in the copy thread itself, so the order is that of the frames shown).
EDITS.append(('        if(frame.layers)return composeOnce(frame);   // section 12\n',
              '        if(frame.layers)return composeOnce(frame);   // section 12\n'
              '        __atomic_store_n(&composeStale,true,__ATOMIC_RELEASE);   // section 13: the compose buffer is no longer on screen\n'))
EDITS.append(('            guamp_scanout_external();   // section 12: the GOP frame is shown again before it is filled\n',
              '            guamp_scanout_external();   // section 12: the GOP frame is shown again before it is filled\n'
              '            __atomic_store_n(&composeStale,true,__ATOMIC_RELEASE);   // section 13\n'))
# 14. Frame pacing (guamp 04/11). The render server had no vsync at all on this port: the refresh tick (sheng-vsync, the original
#     family's type-5 message to the clients queued at +5592) was never on the guamp boot line, so the display links of the apps
#     (UIKit scrolling and animations) had no display timing, and the interface ran at ~10 fps even with the copy at 4-9 ms. Now:
#     - sheng-vsync defaults to 60 (the panel's rate; sheng-vsync=0 turns the tick off, other values as before). The tick only runs
#       while a client has vsync notifications enabled (vsyncWanted), as before.
#     - the tick follows the panel (guamp-vsync-hw, default 1; 0 = plain timer): guamp_display_scan (drivers/NativeEmmc/GuampPacing.inc)
#       reads the INTF_1 frame and line counters of the video-mode timing engine; the vblank that just began is
#       at - line * period / vtotal, the period is measured on the frame counter (at least 30 frames apart, within 10% of the
#       nominal one), the message carries that timestamp and that period, and the next tick is set at the next real vblank. Without
#       readable counters (panel power cycle, clocks off, counter stuck) the tick is the timer of before, at the nominal interval.
#     - swaps keep retiring at once when the validated head is taken (section 10, latest-frame-wins: the copy, the GPU wait and the
#       composition happen afterwards in the copy thread), so the render server never waits on the copy; section 14 measures it.
#     guamp_pacing: [0] swaps accepted, [1] sum of the intervals between accepted swaps under 250 ms (mach ticks), [2] longest such
#     interval since the last report, [3] swaps retired with a known accept time, [4] sum of accept->retire latencies, [5] longest
#     latency since the last report, [6] distinct surfaces in the last 16 accepted swaps (buffers the render server rotates), [7] vsync
#     messages sent, [8] of which on the panel's timing, [9] on the timer, [10] sent more than half a period after the vblank, [11]
#     sum of the delays after the vblank (panel-timed, mach ticks), [12] measured period (mach ticks), [13] vtotal, [14] last frame
#     counter, [15] client queues in use (bit i: the queue at +5512 + 16 i), [16] vsync wanted at the last kick, [17] tick rate (Hz),
#     [18] period measurements, [19] intervals of 250 ms or more (idle gaps), [20] the nominal interval (mach ticks), [21] intervals
#     under 250 ms. drivers/NativeEmmc/GuampPacing.inc logs them every 10 s ('GuampPacing:').
EDITS.append(('__attribute__((weak)) void guamp_scanout_external(void){}}',
              '__attribute__((weak)) void guamp_scanout_external(void){}}\n'
              '// Section 14 (frame pacing): counters, and the INTF_1 counter reader of drivers/NativeEmmc/GuampPacing.inc (weak here: kernels\n'
              '// without it keep the plain timer). 1 = frame, line and vtotal read at mach time *at; 0 = not available.\n'
              'extern "C" {volatile uint64_t guamp_pacing[24];\n'
              '__attribute__((weak)) int guamp_display_scan(uint32_t *,uint32_t *,uint32_t *,uint64_t *){return 0;}}'))
EDITS.append(('''        unsigned hz=0;PE_parse_boot_argn("sheng-vsync",&hz,sizeof(hz));
        if(!hz || hz>240 || !primary || !bootMapping || vsyncTimer)return;''',
              '''        unsigned hz=60;PE_parse_boot_argn("sheng-vsync",&hz,sizeof(hz));   // guamp default 60 (section 14), sheng-vsync=0 off
        guamp_pacing[17]=hz;
        if(!hz || hz>240 || !primary || !bootMapping || vsyncTimer)return;'''))
EDITS.append(('''        nanoseconds_to_absolutetime(1000000000ull/hz,&vsyncInterval);
        vsyncHz=hz;vsyncTimer=timer;native_legacy_vsync_observed(4,hz);''',
              '''        nanoseconds_to_absolutetime(1000000000ull/hz,&vsyncInterval);
        vsyncHz=hz;vsyncTimer=timer;native_legacy_vsync_observed(4,hz);guamp_pacing[20]=vsyncInterval;'''))
EDITS.append(('''        const uint64_t now=mach_absolute_time();
        native_legacy_vsync_notify(this,now,mach_continuous_time(),vsyncInterval);
        native_legacy_vsync_observed(1,vsyncHz);
        vsyncNext+=vsyncInterval;
        if(vsyncNext<=now){vsyncNext=now+vsyncInterval;native_legacy_vsync_observed(3,vsyncHz);}
        vsyncArmed=true;vsyncTimer->wakeAtTime(vsyncNext);''',
              '''        // Section 14: on the panel's own timing when its counters can be read, else the timer as before.
        uint64_t stamp=0,period=0;
        if(guampPanelVblank(&stamp,&period)) {
            const uint64_t abs=mach_absolute_time(),cont=mach_continuous_time();
            native_legacy_vsync_notify(this,stamp,cont-(abs>stamp?abs-stamp:0),period);
            native_legacy_vsync_observed(1,vsyncHz);
            guamp_pacing[7]=guamp_pacing[7]+1;guamp_pacing[8]=guamp_pacing[8]+1;
            const uint64_t late=abs>stamp?abs-stamp:0;
            guamp_pacing[11]=guamp_pacing[11]+late;
            if(late>period/2){guamp_pacing[10]=guamp_pacing[10]+1;native_legacy_vsync_observed(3,vsyncHz);}
            uint64_t next=stamp+period;
            while(next<=abs+period/8)next+=period;   // the next real vblank, never one that is (almost) here already
            vsyncNext=next;
            vsyncArmed=true;vsyncTimer->wakeAtTime(vsyncNext);return;
        }
        const uint64_t now=mach_absolute_time();
        native_legacy_vsync_notify(this,now,mach_continuous_time(),vsyncInterval);
        native_legacy_vsync_observed(1,vsyncHz);
        guamp_pacing[7]=guamp_pacing[7]+1;guamp_pacing[9]=guamp_pacing[9]+1;
        vsyncNext+=vsyncInterval;
        if(vsyncNext<=now){vsyncNext=now+vsyncInterval;native_legacy_vsync_observed(3,vsyncHz);}
        vsyncArmed=true;vsyncTimer->wakeAtTime(vsyncNext);'''))
EDITS.append(('    IOTimerEventSource *vsyncTimer=nullptr;\n',
              r'''    // Section 14: the vblank that just began and the panel's period (mach ticks), from the INTF_1 counters; false = not available.
    int panelTiming=-1;uint32_t panelRefFrame=0;uint64_t panelRefVblank=0,panelPeriod=0;bool panelRef=false;
    bool guampPanelVblank(uint64_t *stamp,uint64_t *period) {
        if(panelTiming<0){unsigned v=1;PE_parse_boot_argn("guamp-vsync-hw",&v,sizeof v);panelTiming=v?1:0;}
        uint32_t frame=0,line=0,vtotal=0;uint64_t at=0;
        if(!panelTiming || guamp_display_scan(&frame,&line,&vtotal,&at)!=1 || !vtotal || line>=vtotal || !at || !vsyncInterval) {
            panelRef=false;return false;
        }
        const uint64_t nominal=vsyncInterval,use=panelPeriod?panelPeriod:nominal;
        const uint64_t back=(uint64_t)line*use/vtotal;
        if(back>=at){panelRef=false;return false;}
        const uint64_t vblank=at-back;
        // The period on the frame counter, from vblank to vblank at least 30 frames apart (a reset counter starts again).
        const uint32_t frames=frame-panelRefFrame;
        if(!panelRef || !frames || frames>100000 || vblank<=panelRefVblank){panelRef=true;panelRefFrame=frame;panelRefVblank=vblank;}
        else if(frames>=30) {
            const uint64_t measured=(vblank-panelRefVblank)/frames;
            if(measured>nominal-nominal/10 && measured<nominal+nominal/10) {
                panelPeriod=panelPeriod?(3*panelPeriod+measured)/4:measured;
                guamp_pacing[12]=panelPeriod;guamp_pacing[18]=guamp_pacing[18]+1;
            }
            panelRefFrame=frame;panelRefVblank=vblank;
        }
        guamp_pacing[13]=vtotal;guamp_pacing[14]=frame;
        *stamp=vblank;*period=panelPeriod?panelPeriod:nominal;return true;
    }
    // Section 14: swaps accepted (rate, intervals, buffers in rotation) and retired (latency from the acceptance).
    struct PacingEntry {const OriginalLegacySwapRequest *request;uint64_t at;};
    PacingEntry pacingEntries[8]={};uint64_t pacingLastAccept=0,pacingIdleGap=0;
    const void *pacingSurfaces[16]={};unsigned pacingSurfaceAt=0;
    void guampPacingAccepted(const OriginalLegacySwapRequest *request) {
        const uint64_t now=mach_absolute_time();
        guamp_pacing[0]=guamp_pacing[0]+1;
        if(!pacingIdleGap)nanoseconds_to_absolutetime(250000000ull,&pacingIdleGap);
        if(pacingLastAccept) {
            const uint64_t gap=now-pacingLastAccept;
            if(gap<pacingIdleGap) {
                guamp_pacing[1]=guamp_pacing[1]+gap;guamp_pacing[21]=guamp_pacing[21]+1;
                if(gap>guamp_pacing[2])guamp_pacing[2]=gap;
            } else guamp_pacing[19]=guamp_pacing[19]+1;
        }
        pacingLastAccept=now;
        // The entry of this request, else a free one, else the oldest.
        PacingEntry *slot=nullptr;
        for(auto &e: pacingEntries)if(e.request==request){slot=&e;break;}
        if(!slot)for(auto &e: pacingEntries)if(!e.request){slot=&e;break;}
        if(!slot){slot=&pacingEntries[0];for(auto &e: pacingEntries)if(e.at<slot->at)slot=&e;}
        *slot={request,now};
        const void *surfaces[4]={};memcpy(surfaces,reinterpret_cast<const unsigned char *>(request)+2624,sizeof(surfaces));
        const void *surface=nullptr;
        for(auto s: surfaces)if(s){surface=s;break;}
        pacingSurfaces[pacingSurfaceAt%16]=surface;pacingSurfaceAt=pacingSurfaceAt+1;
        unsigned distinct=0;
        for(unsigned i=0;i<16;++i) {
            if(!pacingSurfaces[i])continue;
            bool seen=false;
            for(unsigned j=0;j<i && !seen;++j)seen=pacingSurfaces[j]==pacingSurfaces[i];
            if(!seen)distinct=distinct+1;
        }
        guamp_pacing[6]=distinct;
    }
    void guampPacingRetired(const OriginalLegacySwapRequest *request) {
        for(auto &e: pacingEntries)if(e.request==request) {
            const uint64_t latency=mach_absolute_time()-e.at;
            guamp_pacing[3]=guamp_pacing[3]+1;guamp_pacing[4]=guamp_pacing[4]+latency;
            if(latency>guamp_pacing[5])guamp_pacing[5]=latency;
            e={};break;
        }
    }
    // Section 14: which client notification queues are in use (the vsync one is +5592, type 5).
    void guampPacingClients() {
        uint64_t mask=0;
        for(unsigned i=0;i<8;++i) {
            const void *head=nullptr;memcpy(&head,reinterpret_cast<const unsigned char *>(this)+5512+16*i,sizeof(head));
            if(head)mask|=1ull<<i;
        }
        guamp_pacing[15]=mask;guamp_pacing[16]=vsyncWanted()?1:0;
    }
    IOTimerEventSource *vsyncTimer=nullptr;
'''))
EDITS.append(('''    void kickSwapQueue() override {
        armVsync();''', '''    void kickSwapQueue() override {
        guampPacingClients();   // section 14
        armVsync();'''))
EDITS.append(('''        if(isValidated(request))return;
        for(auto &entry: validated)if(!entry){entry=request;return;}''', '''        if(isValidated(request))return;
        guampPacingAccepted(request);   // section 14
        for(auto &entry: validated)if(!entry){entry=request;return;}'''))
EDITS.append(('''        IOMobileFramebufferLegacy::completeCurrentSwap();
        native_legacy_retirement_observed(this);''', '''        IOMobileFramebufferLegacy::completeCurrentSwap();
        native_legacy_retirement_observed(this);
        guampPacingRetired(request);   // section 14'''))
# 14b. (fin44, 04/11) No client ever registered for the vsync message (GuampPacing 'clients 00' all boot), and that is by design:
#     QuartzCore 21H461 (dyld cache) takes the IOMFB vsync path (IOMobileFramebufferGetVSyncRunLoopSource / EnableVSyncNotifications,
#     selector 72 type 5) only when CADeviceUseVBL() is true: CA_FORCE_TIMER set in its environment, or ChipID 0x7000 / 0x8010
#     (___CADeviceUseVBL_block_invoke 0x189582620, bit 1 of IOMFBServer +925, CA::WindowServer::IOMFBServer::add_vsync_source
#     0x189510f9c). Otherwise the render server runs its own display timer at the heartbeat interval
#     (IOMFBDisplay 0x1894f5944: CA_HEARTBEAT_RATE, else the minimum refresh interval read from IOMFBDisplayRefresh, else 1/60 s).
#     The family publishes IOMFBDisplayRefresh only through setDisplayRefreshProperties, which this port called only above 60 Hz,
#     so the render server never had the panel's timing words (minimum, maximum 1/24 s, step). guamp-refresh-props (default 1,
#     0 = as before) publishes them at 60 Hz too, as one fixed rate (maximum = minimum = 1/60 s, the panel has no variable refresh),
#     so the frame-rate server has no lower rate to fall to. guamp_pacing[22] = 1 when they were published.
EDITS.append(('''        unsigned hz=0;PE_parse_boot_argn("sheng-vsync",&hz,sizeof(hz));
        if(primary && hz>60 && hz<=240)native_legacy_set_refresh_properties(this);''',
              '''        unsigned hz=60;PE_parse_boot_argn("sheng-vsync",&hz,sizeof(hz));   // section 14b: 60 Hz published too (guamp-refresh-props)
        if(primary && ((hz>60 && hz<=240) || (hz==60 && guampRefreshProps()))) {
            native_legacy_set_refresh_properties(this);guamp_pacing[22]=1;
        }'''))
EDITS.append(('''        unsigned hz=0;PE_parse_boot_argn("sheng-vsync",&hz,sizeof(hz));
        if(hz<=60 || hz>240)return true;
        unsigned locked=0;PE_parse_boot_argn("sheng-vsync-fixed",&locked,sizeof(locked));''',
              '''        unsigned hz=60;PE_parse_boot_argn("sheng-vsync",&hz,sizeof(hz));
        if(hz<60 || hz>240 || (hz==60 && !guampRefreshProps()))return true;
        unsigned locked=hz==60?1:0;PE_parse_boot_argn("sheng-vsync-fixed",&locked,sizeof(locked));   // 60 Hz: one fixed rate'''))
EDITS.append(('    void guampPacingClients() {\n',
              '''    // Section 14b: IOMFBDisplayRefresh at 60 Hz (guamp-refresh-props, default 1).
    static bool guampRefreshProps() {unsigned v=1;PE_parse_boot_argn("guamp-refresh-props",&v,sizeof v);return v!=0;}
    void guampPacingClients() {
'''))
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

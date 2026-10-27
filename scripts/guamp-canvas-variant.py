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
Without the two arguments nothing changes.
"""
from pathlib import Path
import hashlib,sys
ORIGINAL_SHA256 = '09d7811db8fc6b659e12a05690867326e851f0d11bec7ffbb36b72c33ea035ca'
PATCHED_SHA256 = '310081e050a8321fda160998d7c9dfab798721482f6734a79cead87f8aea67fa'
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
        uint32_t w=0,h=0;unsigned mode=1,filter=1;
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
# 6. The GPU hand-off and the differential copy only move unscaled frames.
EDITS.append(('        if(!gpuBypass && native_legacy_gpu_offer(this,memory,sourceStride,width,height)) {',
              '        if(!gpuBypass && !scaled && native_legacy_gpu_offer(this,memory,sourceStride,width,height)) {'))
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
    print('NativeLegacyDisplay: verified guamp canvas variant (guamp-canvas-width/-height), default unchanged')

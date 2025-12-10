---
title: Image Optimization Guide
sidebar_position: 11
---

# Image Optimization Guide

This guide provides best practices for optimizing images used in the Physical AI & Humanoid Robotics Textbook to ensure fast loading times and optimal user experience.

## Image Format Selection

### PNG Format
Use for:
- Images with transparency
- Simple graphics with limited colors
- Diagrams and screenshots with text
- Line art and illustrations

Benefits:
- Lossless compression
- Supports transparency
- Good for graphics with few colors

### JPEG Format
Use for:
- Photographs and realistic images
- Complex images with many colors
- Images without transparency needs

Benefits:
- Smaller file sizes for complex images
- Good compression for photos
- Universal browser support

### WebP Format (Recommended)
Use for:
- Modern browsers support
- Best compression ratios
- Both lossy and lossless options
- Supports transparency

Benefits:
- 25-35% smaller than JPEG/PNG
- Superior compression quality
- Supports both lossy and lossless

## Image Dimensions and Resolution

### Web Display Resolution
- Use 72-96 DPI for web display
- Set actual pixel dimensions needed
- Avoid large images scaled down with CSS
- Consider responsive design needs

### Common Dimensions
- **Full-width images**: 1200-1920px wide
- **Content images**: 800-1200px wide
- **Diagrams**: 600-1000px wide
- **Icons**: 16-256px (vector formats preferred)

## File Size Optimization

### Target File Sizes
- **Simple diagrams**: < 50KB
- **Complex diagrams**: < 200KB
- **Photographs**: < 300KB
- **Screenshots**: < 150KB

### Optimization Techniques
1. **Compression**: Use appropriate quality settings
2. **Cropping**: Remove unnecessary areas
3. **Resolution**: Match display requirements
4. **Format conversion**: Choose optimal format

## Optimization Tools

### Online Tools
- TinyPNG: https://tinypng.com/
- Compressor.io: https://compressor.io/
- ImageOptim: https://imageoptim.com/

### Command Line Tools
```bash
# Optimize PNG files
pngquant --quality=65-80 *.png

# Optimize JPEG files
jpegoptim --max=80 *.jpg

# Convert to WebP
cwebp -q 80 input.jpg -o output.webp
```

### ImageMagick
```bash
# Resize and optimize
convert input.png -resize 800x600 -quality 85 output.jpg

# Strip metadata
convert input.png -strip output.png
```

## Responsive Images

### Multiple Sizes
Provide different image sizes for different screen densities:

```markdown
![Description](/img/image-800w.jpg) <!-- Standard resolution -->
<!-- Alternative: /img/image-1600w.jpg for high-DPI displays -->
```

### Lazy Loading
Images are automatically lazy-loaded by Docusaurus for better performance.

## Accessibility Considerations

### Alt Text
- Write descriptive alt text for all images
- Keep it concise but informative
- Describe the content and function
- Omit "Image of..." prefix

### Captions
- Include figure numbers and descriptions
- Provide context for complex diagrams
- Reference images in the text
- Maintain accessibility standards

## Diagram Optimization

### Vector Formats
- Use SVG for diagrams when possible
- Editable and scalable without quality loss
- Smaller file sizes for simple graphics
- Good for technical diagrams

### Export Settings for Diagrams
- Use appropriate resolution (72-96 DPI)
- Optimize SVG files by removing unnecessary metadata
- Use CSS for styling instead of inline styles
- Minimize file size while maintaining clarity

## Performance Guidelines

### Loading Strategy
1. **Above-the-fold**: Optimize for fastest loading
2. **Below-the-fold**: Optimize for smallest file size
3. **Large images**: Consider using progressive loading

### File Organization
- Store in `/static/img/` directory
- Use descriptive, lowercase filenames
- Organize by content area if needed
- Use consistent naming conventions

Example: `ros-architecture-diagram.png`

## Testing and Validation

### Performance Testing
- Test loading times on different connections
- Verify image quality is acceptable
- Check responsive behavior
- Validate accessibility features

### Tools for Testing
- Chrome DevTools Network tab
- PageSpeed Insights
- WebPageTest
- Lighthouse audits

## Automation

### Build-time Optimization
Consider adding image optimization to your build process:

```javascript
// Example webpack configuration
module.exports = {
  module: {
    rules: [
      {
        test: /\.(png|jpe?g|gif)$/i,
        use: [
          {
            loader: 'image-webpack-loader',
            options: {
              mozjpeg: { progressive: true, quality: 80 },
              optipng: { enabled: false },
              pngquant: { quality: [0.6, 0.8], speed: 4 },
              gifsicle: { interlaced: false },
            },
          },
        ],
      },
    ],
  },
};
```

Following these guidelines will ensure that images in the Physical AI & Humanoid Robotics Textbook load quickly and look great across all devices and browsers.
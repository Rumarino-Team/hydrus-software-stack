# SAM2 Data Engine - YOLO Dataset Generator

An automatic data engine GUI application that uses visual prompts to interact with SAM2 (Segment Anything Model 2) for generating YOLO training datasets with automatic video segmentation.

## Features

- **Interactive GUI**: Built with PySide6 for modern and intuitive user experience
- **SAM2 Integration**: Uses Ultralytics SAM2 for automatic segmentation
- **Video Navigation**: Move back and forth through video frames
- **Point-based Prompting**: Add positive/negative points for precise segmentation
- **Automatic Propagation**: Forward and backward mask propagation across frames
- **Smart Caching**: Cache frames, masks, and SAM2 features for faster processing
- **YOLO Export**: Export annotations in YOLO dataset format
- **Object Classification**: Assign class names to segmented objects
- **Real-time Visualization**: See segmentation results with overlay masks

## Requirements

- Python 3.8+
- PySide6
- OpenCV
- NumPy
- PyTorch
- Ultralytics (for SAM2)
- Pillow

## Installation

1. **Clone or navigate to the data_engine directory**:
```bash
cd /path/to/hydrus-software-stack/data_engine
```

2. **Install dependencies**:
```bash
pip install -r requirements.txt
```

3. **Run setup script** (optional - downloads SAM2 models):
```bash
python setup.py
```

4. **Launch the application**:
```bash
python main.py
```

## Usage

### 1. Load Video
- Click "Load Video" to select a video file
- Supported formats: MP4, AVI, MOV, MKV, WMV
- Set output directory for caching and exports

### 2. Navigate Frames
- Use the slider or spinbox to navigate between frames
- Use "◀ Prev" and "Next ▶" buttons for frame-by-frame navigation
- Frames are automatically cached for faster access

### 3. Add Object Classes
- In the right panel, add class names for your objects
- Each class gets a unique ID automatically
- Select a class before segmenting objects

### 4. Segment Objects
- **Add Points**: Click on the frame to add positive (green) or negative (red) points
- **Positive Points**: Click inside the object you want to segment
- **Negative Points**: Click outside the object or on background
- **Segment**: Click "Segment Object" to run SAM2 segmentation
- **Clear Points**: Remove all points and start over

### 5. Propagate Masks
- After segmenting an object, use propagation to track it across frames
- **Forward Propagation**: Tracks object in subsequent frames
- **Backward Propagation**: Tracks object in previous frames

### 6. Export Dataset
- Click "Export YOLO Dataset" to save annotations
- Creates folder structure: `images/`, `labels/`, `classes.txt`, `dataset.yaml`
- Annotations are saved in YOLO segmentation format

## Directory Structure

```
data_engine/
├── main.py              # Main GUI application
├── sam_processor.py     # SAM2 integration module
├── utils.py            # Utility functions
├── setup.py            # Setup script
├── requirements.txt    # Python dependencies
├── README.md           # This file
├── cache/              # Cached data
│   ├── frames/         # Cached video frames
│   ├── masks/          # Cached segmentation masks
│   └── features/       # Cached SAM2 features
├── projects/           # Project configurations
└── exports/            # Exported datasets
```

## Output Format

The tool exports datasets in YOLO format:

### Dataset Structure
```
yolo_dataset/
├── images/             # Training images
│   ├── frame_000001.jpg
│   ├── frame_000002.jpg
│   └── ...
├── labels/             # Segmentation labels
│   ├── frame_000001.txt
│   ├── frame_000002.txt
│   └── ...
├── classes.txt         # Class names
└── dataset.yaml        # YOLO configuration
```

### Label Format
Each `.txt` file contains one line per object:
```
class_id x1 y1 x2 y2 x3 y3 ... xn yn
```
Where coordinates are normalized (0-1) polygon points.

## SAM2 Models

The application supports different SAM2 model sizes:
- `sam2_s.pt` - Small (fastest)
- `sam2_b.pt` - Base (balanced)
- `sam2_l.pt` - Large (most accurate)

Models are automatically downloaded on first use.

## Performance Tips

1. **Use GPU**: Ensure CUDA is available for faster inference
2. **Cache Features**: Enable feature caching for repeated processing
3. **Optimize Points**: Use minimal points for faster segmentation
4. **Batch Processing**: Process multiple frames at once when possible

## Troubleshooting

### Common Issues

1. **Model Loading Error**:
   - Ensure internet connection for model download
   - Check CUDA/PyTorch compatibility
   - Try different model sizes

2. **Memory Issues**:
   - Use smaller SAM2 model (sam2_s.pt)
   - Reduce video resolution
   - Clear cache regularly

3. **Slow Performance**:
   - Enable GPU acceleration
   - Use feature caching
   - Process frames in batches

### System Requirements

- **Minimum**: 8GB RAM, CPU-only
- **Recommended**: 16GB+ RAM, NVIDIA GPU with 6GB+ VRAM
- **For Large Videos**: 32GB+ RAM, High-end GPU

## Advanced Features

### Custom Prompting
- Combine point and box prompts for complex objects
- Use multiple positive/negative points for precision
- Adjust SAM2 parameters for different object types

### Batch Processing
- Process multiple videos sequentially
- Export datasets in different formats
- Automated quality control and validation

### Integration
- Export to other annotation formats (COCO, Pascal VOC)
- Integration with training pipelines
- Custom post-processing workflows

## Contributing

This tool is part of the Hydrus Software Stack. For contributions:

1. Follow the existing code structure
2. Add tests for new features
3. Update documentation
4. Ensure compatibility with SAM2 updates

## License

See the main Hydrus Software Stack license for details.

## References

- [Ultralytics SAM2 Documentation](https://docs.ultralytics.com/models/sam-2/)
- [Segment Anything Model 2](https://github.com/facebookresearch/segment-anything-2)
- [YOLO Format Specification](https://docs.ultralytics.com/datasets/segment/)

## Support

For issues and questions:
1. Check the troubleshooting section
2. Review SAM2 documentation
3. Open an issue in the main repository

#!/usr/bin/env python3

import os
import argparse
from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import matplotlib.image as mpimg

def generate_pdf_report(images_dir, report_md, output_pdf):
    """Generate a PDF report with benchmark visualizations and text."""
    # Read the markdown report
    with open(report_md, 'r') as f:
        report_text = f.read()
    
    # Get all PNG images in the directory
    image_files = sorted([f for f in os.listdir(images_dir) if f.endswith('.png')])
    
    # Create PDF
    with PdfPages(output_pdf) as pdf:
        # Title page
        plt.figure(figsize=(8.5, 11))
        plt.axis('off')
        plt.text(0.5, 0.5, 'ASV Energy-Aware Planning\nBenchmark Results', 
                 ha='center', va='center', fontsize=24)
        plt.text(0.5, 0.4, 'Generated Analysis Report', 
                 ha='center', va='center', fontsize=16)
        pdf.savefig()
        plt.close()
        
        # Add report text as a page
        plt.figure(figsize=(8.5, 11))
        plt.axis('off')
        
        # Split report content by section headers
        sections = report_text.split('## ')
        
        # Add title (first section)
        title = sections[0].strip()
        plt.text(0.5, 0.95, title, ha='center', va='top', fontsize=14, fontweight='bold')
        
        # Add summary text (first few paragraphs)
        summary_text = '\n'.join(sections[1].split('\n')[:6]) if len(sections) > 1 else ""
        plt.text(0.1, 0.85, 'Summary:\n\n' + summary_text, 
                 ha='left', va='top', fontsize=10, wrap=True)
        
        plt.text(0.5, 0.1, 'See full report and visualizations in the following pages', 
                 ha='center', va='bottom', fontsize=10, fontstyle='italic')
        
        pdf.savefig()
        plt.close()
        
        # Add each image as a separate page with appropriate title
        for img_file in image_files:
            plt.figure(figsize=(11, 8.5))  # Landscape
            
            # Format title from filename
            parts = img_file.replace('.png', '').split('_')
            scenario_type = parts[0].title()
            metric = ' '.join([p.title() for p in parts[1:]])
            
            plt.title(f"{scenario_type} Scenario: {metric}")
            
            # Load and display image
            img = mpimg.imread(os.path.join(images_dir, img_file))
            plt.imshow(img)
            plt.axis('off')
            
            pdf.savefig()
            plt.close()
    
    print(f"PDF report generated: {output_pdf}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Generate PDF report from benchmark results')
    parser.add_argument('--images_dir', type=str, required=True,
                       help='Directory containing visualization images')
    parser.add_argument('--report_md', type=str, required=True,
                       help='Markdown report file')
    parser.add_argument('--output_pdf', type=str, required=True,
                       help='Output PDF file path')
    args = parser.parse_args()
    
    generate_pdf_report(args.images_dir, args.report_md, args.output_pdf) 
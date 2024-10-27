#include <hpdf.h>
#include <string>
#include <iostream>

class PDFGenerator {
private:
    HPDF_Doc pdf;
    HPDF_Page page;
    HPDF_Font font;
    std::string outputFilename;

public:
    // Default constructor
    PDFGenerator() : outputFilename("default_output.pdf") {
        initialize();
    }

    // Constructor with filename
    PDFGenerator(const std::string& filename) : outputFilename(filename) {
        initialize();
    }

private:
    void initialize() {
        pdf = HPDF_New(NULL, NULL);
        if (!pdf) {
            throw std::runtime_error("Failed to create PDF object");
        }

        // Error handler
        HPDF_SetErrorHandler(pdf, error_handler);

        // Add a new page
        page = HPDF_AddPage(pdf);
        HPDF_Page_SetSize(page, HPDF_PAGE_SIZE_A4, HPDF_PAGE_PORTRAIT);

        // Set font
        font = HPDF_GetFont(pdf, "Helvetica", NULL);
        HPDF_Page_SetFontAndSize(page, font, 12);
    }

public:
    void addText(float x, float y, const std::string& text) {
        HPDF_Page_BeginText(page);
        HPDF_Page_TextOut(page, x, y, text.c_str());
        HPDF_Page_EndText(page);
    }

    void save(const std::string& filename = "") {
        const std::string& fileToUse = filename.empty() ? outputFilename : filename;
        HPDF_SaveToFile(pdf, fileToUse.c_str());
    }

    ~PDFGenerator() {
        if (pdf) {
            HPDF_Free(pdf);
        }
    }

private:
    static void error_handler(HPDF_STATUS error_no, HPDF_STATUS detail_no, void* /* user_data */) {
        std::cerr << "PDF Error: error_no=" << (unsigned int)error_no 
                  << ", detail_no=" << (unsigned int)detail_no << std::endl;
        throw std::runtime_error("PDF Error occurred");
    }
};
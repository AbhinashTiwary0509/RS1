#include <hpdf.h>
#include <string>
#include <iostream>
#include <filesystem>

class PDFGenerator {
private:
    HPDF_Doc pdf;
    HPDF_Page page;
    HPDF_Font font;
    std::string outputFilename;

public:
    // Default constructor
    PDFGenerator() : pdf(nullptr), page(nullptr), font(nullptr), outputFilename("default_output.pdf") {
        initialize();
    }

    // Delete copy constructor and assignment operator since we're managing resources
    PDFGenerator(const PDFGenerator&) = delete;
    PDFGenerator& operator=(const PDFGenerator&) = delete;

private:
    void initialize() {
        // Create a new PDF document
        pdf = HPDF_New(error_handler, NULL);
        if (!pdf) {
            throw std::runtime_error("Failed to create PDF object");
        }

        try {
            // Add a new page
            page = HPDF_AddPage(pdf);
            if (!page) {
                throw std::runtime_error("Failed to add page");
            }

            HPDF_Page_SetSize(page, HPDF_PAGE_SIZE_A4, HPDF_PAGE_PORTRAIT);

            // Set font
            font = HPDF_GetFont(pdf, "Helvetica", NULL);
            if (!font) {
                throw std::runtime_error("Failed to load font");
            }

            HPDF_Page_SetFontAndSize(page, font, 12);
        }
        catch (...) {
            if (pdf) {
                HPDF_Free(pdf);
                pdf = nullptr;
            }
            throw;
        }
    }

public:
    void updateOutputFileName(const std::string& filename) {
        if (filename.empty()) {
            throw std::invalid_argument("Filename cannot be empty");
        }
        outputFilename = filename;
    }

    void clear() {
        if (pdf) {
            HPDF_Free(pdf);
            pdf = nullptr;
        }
        initialize();  // Reinitialize the PDF document
    }

    void addText(float x, float y, const std::string& text) {
        if (!pdf || !page) {
            throw std::runtime_error("PDF not properly initialized");
        }

        HPDF_Page_BeginText(page);
        HPDF_Page_TextOut(page, x, y, text.c_str());
        HPDF_Page_EndText(page);
    }

    void save(const std::string& filename = "") {
        deleteFile();
        if (!pdf) {
            throw std::runtime_error("PDF not properly initialized");
        }

        const std::string& fileToUse = filename.empty() ? outputFilename : filename;
        HPDF_SaveToFile(pdf, fileToUse.c_str());
    }

    bool deleteFile() {
        try {
            if (std::filesystem::exists(outputFilename)) {
                return std::filesystem::remove(outputFilename);
            }
            return false;
        } catch (const std::filesystem::filesystem_error& e) {
            std::cerr << "Filesystem error: " << e.what() << std::endl;
            return false;
        }
    }

    ~PDFGenerator() {
        if (pdf) {
            HPDF_Free(pdf);
            pdf = nullptr;
        }
    }

private:
    static void error_handler(HPDF_STATUS error_no, HPDF_STATUS detail_no, void* /* user_data */) {
        std::cerr << "PDF Error: error_no=" << (unsigned int)error_no 
                  << ", detail_no=" << (unsigned int)detail_no << std::endl;
        throw std::runtime_error("PDF Error occurred");
    }
};
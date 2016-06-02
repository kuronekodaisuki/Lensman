//
// Ref:
//	http://www.gtkmm.org/en/
//	http://stackoverflow.com/questions/9299187/adding-an-opengl-window-into-a-gtk-builder
//	http://stackoverflow.com/questions/1904481/opengl-gtkglextmm-glade
//
// I.e. I'm simply loading a UI that contains an empty container widget, 
// get the handle to that by name, then create the GL-enabled drawing area in code 
// and add it to the empty container.
//  Supposedly, it's possible to "load-time" substitute an ordinary Gtk DrawingArea 
// for a GtkGL one (see this posting) but that method didn't work for me; the above, 
// programmatically creating it, always did.

You need gtkglext / gtkglextmm for the GL Drawingarea widget.

#include <gtkmm.h>
#include <gtkglmm.h>

class GLWidget : public Gtk::GL::DrawingArea 
{
	public:
	GLWidget(Glib::RefPtr<Gdk::GL::Config> glconfig)
		: Gtk::GL::DrawingArea(glconfig) {}
	~GLWidget() {}
	virtual bool on_expose_event(GdkEventExpose* event);
};

bool GLWidget::on_expose_event(GdkEventExpose* event)
{
	Glib::RefPtr<Gdk::GL::Drawable> d = get_gl_drawable();
	d->gl_begin(get_gl_context());

	// make this as complex as you need
	glClear(GL_COLOR_BUFFER_BIT);

	d->swap_buffers();
	d->gl_end();
	return true;
}

int main(int argc, char **argv)
{
	Gtk::Main kit(argc, argv);
	Gtk::GL::init(argc, argv);

	Glib::RefPtr<Gtk::Builder> builder = Gtk::Builder::create_from_file("ui.glade");

	Gtk::Window* mainWindow;
	Gtk::Alignment* container;

	builder->get_widget("mainWindow", mainWindow);
	builder->get_widget("Box", container);

	if (mainWindow == NULL || container == NULL) {
		g_critical("Gtk Builder failed to load mainWindow and/or container !\n");
		return -1;
	}

	Glib::RefPtr<Gdk::GL::Config> glconfig;

	glconfig = Gdk::GL::Config::create(Gdk::GL::MODE_RGBA | Gdk::GL::MODE_DOUBLE);
	if (!glconfig)
		glconfig = Gdk::GL::Config::create(Gdk::GL::MODE_RGB);
	if (!glconfig) {
		g_critical("Cannot create OpenGL-capable config\n");
		return -1;
	}

	GLWidget drawingArea(glconfig);

	drawingArea.set_size_request(640, 480);
	drawingArea.show();
	container->add(drawingArea);

	kit.run(*mainWindow);

	return 0;
}
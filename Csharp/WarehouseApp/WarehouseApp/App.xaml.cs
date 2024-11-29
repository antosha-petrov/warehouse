namespace WarehouseApp
{
    public partial class App : Application
    {
        public App()
        {
            InitializeComponent();
        }

        protected override Window CreateWindow(IActivationState? activationState)
        {
            var page = new Window
            {
                Page = new MainPage()
            };
            return page;
        }
    }
}
namespace WarehouseApp
{
    public partial class App : Application
    {
        public App()
        {
            InitializeComponent();
            Routing.RegisterRoute(nameof(OrderPage), typeof(OrderPage));
        }

        protected override Window CreateWindow(IActivationState? activationState)
        {
            return new Window
            {
                Page = new AppShell()
            };
        }
    }
}